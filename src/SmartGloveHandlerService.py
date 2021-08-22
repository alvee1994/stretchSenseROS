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


'''
Imports related to ros action server
'''
import roslib

roslib.load_manifest('stretchsense')
import threading
from multiprocessing.pool import ThreadPool
import rospy
import actionlib
import time
from stretchsense.msg import smartGloveServerAction, smartGloveServerResult, smartGloveServerGoal, smartGloveServerFeedback

class SmartGloveHandlerService():
    rospack = rospkg.RosPack()
    package_directory = rospack.get_path('stretchsense')

    def __init__(self):
        # trained data location
        self.haveTheta = False
        self.peripheralInUse = None
        self.ros_publish_rate = rospy.Rate(100)

        self.knownPeripherals = {}
        self.availablePeripherals = []

        self.params = {'hci': 0,
                       'timeout': 4,
                       'sensitivity': -128,
                       'discover': True,
                       'all': True,
                       'new': True,
                       'verbose': True}


        '''
        Variables for server
        '''
        self.sensor_read_thread = None
        self.stop_running_thread = False
        self.feedback = smartGloveServerFeedback()
        self.result = smartGloveServerResult()
        self.res = []
        self.server = actionlib.SimpleActionServer('smartglove', smartGloveServerAction, self.execute, False)
        self.server.start()

    def find_gloves(self):
        scanner = btle.Scanner().withDelegate(StretchSenseDelegate(self.params))
        devices = scanner.scan(3)
        self.availablePeripherals = self.list_available_peripherals(devices)
        self.knownPeripherals = self.list_known_peripherals()

        if len(self.availablePeripherals) > 0:
            return True
        else:
            print('No compatible gloves found\n')
            return False

    def list_available_peripherals(self, devices)->List:
        listOfPeripheralsAvailable = []
        for device in devices:
            for (sdid, desc, val) in device.getScanData():
                if val == 'StretchSense':
                    listOfPeripheralsAvailable.append(device.addr)

        return listOfPeripheralsAvailable

    def list_known_peripherals(self):
        # compare to list of known peripherals
        knownPeripheralsYaml = open(f'{self.package_directory}/src/data/knownPeripherals.yaml')
        kP = yaml.load(knownPeripheralsYaml, Loader=yaml.FullLoader)
        return kP['Gloves']

    def select_glove(self):
        print("Select a glove to connect\n")
        for idx, glove in enumerate(self.availablePeripherals):
            if glove in self.knownPeripherals.keys():
                print(f'\t {idx}. {self.knownPeripherals[glove]}\n')
            else:
                print(f'\t {idx} New Peripheral, address: {glove}\n')

        selectedIdx = int(input(f'Select from 0 to {len(self.availablePeripherals) - 1} for a glove: '))
        addr = self.availablePeripherals[selectedIdx]
        return addr

    def action_server_connect_glove(self, selection):
        # if the user returns an address, it must be available but unknown peripheral
        if selection in self.availablePeripherals:
            self.connect_glove(selection)
            return True
        else:
            # if the above is not satisfied, the selection must be a name of a known peripheral
            for key, value in self.knownPeripherals.items():
                if value == selection:
                    self.connect_glove(key)
                    return True
                else:
                    pass

        return False


    def connect_glove(self, address):
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

    def find_trained_data(self):
        files = os.listdir(f'{self.package_directory}/src/data/')
        filenames = [file for file in files if file[-4:] == '.csv']
        return filenames

    def action_server_select_data(self, file):
        filename = f'{self.package_directory}/src/data/{file}'
        return self.select_data_file(filename)

    def select_data_file(self, filename):
        theta = pd.read_csv(filename, sep=',', header=None)
        if isinstance(self.peripheralInUse, btle.Peripheral):
            self.peripheralInUse.delegate.setTheta(theta.values)
            return True
        else:
            return False

    def select_data(self, trainedData):
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
            self.select_data_file(filename)

            return True
                # self.peripheralInUse.delegate.publishCapacitance()

    def read_sensor(self):
        if isinstance(self.peripheralInUse, btle.Peripheral):
            if self.peripheralInUse.waitForNotifications(1.0):
                return

    def calibrate_glove(self):
        while not rospy.is_shutdown() or self.peripheralInUse.delegate.trainingDone:
            if self.stop_running_thread:
                break

            theta = self.peripheralInUse.delegate.calibrateSensors(self.read_sensor)
            if theta is not None:
                filename = self.package_directory + "/src/data/theta_" + str(rospy.Time.now()) + ".csv"
                theta = pd.DataFrame(theta)
                theta.to_csv(filename, index=False, header=False)
                print('saved new model')
                self.haveTheta = True
                self.call_publisher()
            else:
                self.ros_publish_rate.sleep()

    def call_publisher(self):
        while not rospy.is_shutdown():
            if self.stop_running_thread:
                break
            self.peripheralInUse.delegate.publishCapacitance(callback=self.read_sensor)
            self.ros_publish_rate.sleep()


    def scan(self) -> list:
        # return the list of devices found
        self.feedback.feedback = 'scanning...'
        if self.find_gloves():
            kP = self.knownPeripherals
            aP = self.availablePeripherals
            known_peripherals = [value for value in kP.values()]
            return known_peripherals + aP
        else:
            return ['No compatible gloves found']

    def connect(self, addr: str) -> bool:
        return self.action_server_connect_glove(addr)

    def stop_thread(self):
        self.stop_running_thread = True
        if isinstance(self.sensor_read_thread, threading.Thread):
            self.sensor_read_thread.join()

    def start_thread(self, function: callable):
        self.sensor_read_thread = threading.Thread(target=function, args=())
        self.sensor_read_thread.start()

    def execute(self, goal: smartGloveServerGoal) -> None:
        received = goal.goal
        thegoal = received[0]
        carry = received[-1]

        if thegoal == 'scan':
            self.res = self.scan()

        elif thegoal == 'connect':
            self.stop_thread()
            self.disconnect_peripheral()

            if self.connect(carry):
                data_files = self.find_trained_data()
                self.res = ['Connection Successful!'] + data_files
            else:
                self.res = ['Connection Failed']

        elif thegoal == 'select_theta':
            if self.action_server_select_data(carry):
                self.start_thread(self.call_publisher)
                # self.call_publisher()
                self.res = ['Model Selected']
            else:
                self.res = ['Error selecting file']

        elif thegoal == 'calibrate':
            self.start_thread(self.calibrate_glove)
            # self.calibrate_glove()
            self.res = ['Clench your fist and follow the model on the screen']

        elif thegoal == 'stop':
            print('called stop')
            self.stop_thread()
            self.res = ['thread stopped']

        elif thegoal == 'disconnect':
            print('disconnecting')
            self.disconnect_peripheral()
            self.res = ['disconnected']

        self.result.result = self.res
        self.server.set_succeeded(self.result)

    def disconnect_peripheral(self):
        if isinstance(self.peripheralInUse, btle.Peripheral):
            self.peripheralInUse.disconnect()

if __name__ == '__main__':
    try:
        rospy.init_node('stretchsenseCAP')
        server = SmartGloveHandlerService()
        rospy.spin()
    except KeyboardInterrupt as e:
        print(e)
        exit()