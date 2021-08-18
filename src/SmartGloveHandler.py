#!/usr/bin/python3

import rospkg, rospy
from typing import List
from bluepy import btle
import yaml
import pandas as pd
import time
import os
from lib.StretchSenseDelegate import StretchSenseDelegate
# from lib import TrainingData, SolveLeastSquares


class SmartGloveHandler:
    rospack = rospkg.RosPack()
    package_directory = rospack.get_path('stretchsense')

    def __init__(self):
        # trained data location
        self.haveTheta = False
        self.peripheralInUse = None

        self.params = {'hci': 0,
                       'timeout': 4,
                       'sensitivity': -128,
                       'discover': True,
                       'all': True,
                       'new': True,
                       'verbose': True}

    def findGloves(self):
        scanner = btle.Scanner().withDelegate(StretchSenseDelegate(self.params))
        devices = scanner.scan(3)
        availblePeripherals = self.listAvailablePeripherals(devices)
        knownPeripherals = self.listKnownPeripherals()

        if len(availblePeripherals) > 0:
            return availblePeripherals, knownPeripherals
        else:
            print('No compatible gloves found\n')
            quit()

    def listAvailablePeripherals(self, devices)->List:
        listOfPeripheralsAvailable = []
        for device in devices:
            for (sdid, desc, val) in device.getScanData():
                if val == 'StretchSense':
                    listOfPeripheralsAvailable.append(device.addr)

        return listOfPeripheralsAvailable

    def listKnownPeripherals(self):
        # compare to list of known peripherals
        knownPeripherals = open(f'{self.package_directory}/src/data/knownPeripherals.yaml')
        kP = yaml.load(knownPeripherals, Loader=yaml.FullLoader)
        return kP['Gloves']

    def selectGlove(self, availablePeripherals, knownPeripherals):
        print("Select a glove to connect\n")
        for idx, glove in enumerate(availablePeripherals):
            if glove in knownPeripherals.keys():
                print(f'\t {idx}. {knownPeripherals[glove]}\n')
            else:
                print(f'\t {idx} New Peripheral, address: {glove}\n')

        selectedIdx = int(input(f'Select from 0 to {len(availablePeripherals) - 1} for a glove: '))
        addr = availablePeripherals[selectedIdx]
        return addr


    def connectGlove(self, address):
        print(f'connecting to glove with address {address}\n')
        p = btle.Peripheral(address, 'random')
        p.withDelegate(StretchSenseDelegate(p))
        print(f'connected to {address}\n')

        # enable ble notifications and change sampling rate
        svc = p.getServiceByUUID('00001701-7374-7265-7563-6873656e7365')
        char = svc.getCharacteristics()[0]
        handle = char.valHandle
        p.writeCharacteristic(handle + 1, b'\x01\x00')  # turn on notifications
        p.writeCharacteristic(29, b'\x5a')  # change sampling rate to 90Hz
        self.peripheralInUse = p
        return True

    def findTrainedData(self):
        files = os.listdir(f'{self.package_directory}/src/data/')
        data = [file for file in files if file[-4:] == '.csv']
        return data


    def selectData(self, trainedData):
        print("\nSelect a model to use\n")
        for idx, data in enumerate(trainedData):
            print(f'\t{idx}. {data}\n')

        selected = input(f'\n0 to {len(trainedData) - 1} or Y to calibrate: ')

        if selected in ['Y', 'y']:
            print('Getting ready to Calibrate in 5 secs')
            time.sleep(5)

            return False
        elif selected.isdigit():
            idx = int(selected)
            filename = f'{self.package_directory}/src/data/{trainedData[idx]}'
            theta = pd.read_csv(filename, sep=',', header=None)

            if isinstance(self.peripheralInUse, btle.Peripheral):
                self.peripheralInUse.delegate.setTheta(theta.values)

            return True
                # self.peripheralInUse.delegate.publishCapacitance()

    def readSensor(self):
        if isinstance(self.peripheralInUse, btle.Peripheral):
            if self.peripheralInUse.waitForNotifications(1.0):
                return

    def calibrateGlove(self):
        theta = self.peripheralInUse.delegate.calibrateSensors(self.readSensor)
        filename = self.package_directory + "/src/data/theta_" + str(rospy.Time.now()) + ".csv"
        theta = pd.DataFrame(theta)
        theta.to_csv(filename, index=False, header=False)
        print('saved new model')
        return True


    def callPublisher(self):
        self.peripheralInUse.delegate.publishCapacitance(callback=self.readSensor)


if __name__ == "__main__":
    rospy.init_node('stretchsenseCAP', anonymous=True)

    myglove = SmartGloveHandler()
    aP, kP = myglove.findGloves()
    addr = myglove.selectGlove(aP,kP)
    if myglove.connectGlove(addr):
        data = myglove.findTrainedData()
        if myglove.selectData(data):
            myglove.peripheralInUse.delegate.publishCapacitance(myglove.readSensor)
        else:
            myglove.calibrateGlove()
            myglove.peripheralInUse.delegate.publishCapacitance(myglove.readSensor)