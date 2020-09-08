#!/usr/bin/env python3

import time
import numpy as np
import yaml
import pandas as pd
from bluepy import btle
import binascii
import os.path
from lib import TrainingData, SolveLeastSquares #, StretchSense
from bluepy.btle import Scanner, DefaultDelegate
import os

# ROS
import rospy
from collections import deque
from sensor_msgs.msg import JointState
from stretchsense.msg import ssCap



class StretchSenseDelegate(btle.DefaultDelegate):
    def __init__(self, params):
        btle.DefaultDelegate.__init__(self)
        self.val = [0,1,2,3]


    def handleNotification(self, cHandle, data):
        decimalValue = (binascii.b2a_hex(data))
        splitted = [decimalValue[i:i+4] for i in range(0, len(decimalValue),4)]
        self.val = np.array(list(map(lambda x: int((x),16)/10, splitted)))
        idx = np.nonzero(self.val)
        self.val = self.val[idx]
        # print(self.val)
        SmartGloveSS.capacitance = self.val

class SmartGloveSS():

    """
    Variables to check old calibration data
    """
    haveTheta = False
    thetafile = "/home/" + os.getlogin() + "/borealis_ws/src/stretchsense/src/theta_new.csv"

    """
    More Variables
    """
    toRad = 0.01745329251

    """
    the initiated peripheral itself
    """
    peripheralInUse = deque(maxlen=1)
    capacitance = []

    def __init__(self):
        # machine learning
        self.Solver = SolveLeastSquares.SolveLeastSquares()
        self.Training = TrainingData.TrainingData()

        # stretchsense API
        # self.StretchSenseObject = StretchSense.StretchSenseAPI()
        self.pubjs = rospy.Publisher('/joint_states', JointState, queue_size=2)
        self.Joints = JointState()
        self.Position = ssCap()
        self.rate = rospy.Rate(200)
        self.mtheta = []
        # get training data
        self.trainingY = self.Training.getTrainingData()

        self.Joints.header.frame_id = "ssFingers"
        self.Joints.name = ["metacarpal_thumb_splay_2", "thumb_meta_prox",
                            "thumb_prox_inter", "metacarpal_index", "index_prox_inter", "index_inter_dist",
                            "metacarpal_middle", "middle_prox_inter", "middle_inter_dist",
                            "metacarpal_ring",
                            "ring_prox_inter", "ring_inter_dist", "metacarpal_pinky", "pinky_prox_inter",
                            "pinky_inter_dist"]

    def connectGlove(self):
        # initiate scanner to scan and filter stretchsense devices with the following parameters
        params = {'hci': 0,
                  'timeout': 4,
                  'sensitivity': -128,
                  'discover': True,
                  'all': True,
                  'new': True,
                  'verbose': True}

        scanner = Scanner().withDelegate(StretchSenseDelegate(params))
        devices = scanner.scan(3)
        listOfPeripheralsAvailable = []

        for dev in devices:
            for (sdid, desc, val) in dev.getScanData():
                if val == 'StretchSense':
                    listOfPeripheralsAvailable.append(dev.addr)

        # get name of known peripherals from yaml file
        knownPeripherals = open("/home/" + os.getlogin() + "/borealis_ws/src/stretchsense/src/knownPeripherals.yaml")
        kP = yaml.load(knownPeripherals, Loader=yaml.FullLoader)
        Gloves = kP['Gloves']

        # user selects a glove to connect
        if len(listOfPeripheralsAvailable) > 0:
            print('Select a glove to connect\n')
            for i in range(len(listOfPeripheralsAvailable)):
                if listOfPeripheralsAvailable[i] in Gloves.keys():
                    print('\t %i. %s' % (i, Gloves[listOfPeripheralsAvailable[i]]))
                else:
                    print('\t %i. Unknown, addr: %s' % (i, listOfPeripheralsAvailable[i]))
        else:
            print('no gloves found')

        selected = int(input('\nSelect from 0 to %i for a glove ' % (len(listOfPeripheralsAvailable) - 1)))
        addr = listOfPeripheralsAvailable[selected]
        print('connecting to addr %s' % addr)
        self.connectOnePeripheral(addr)
        return True

    def connectOnePeripheral(self, addr):
        p = btle.Peripheral(addr, 'random')
        p.withDelegate(StretchSenseDelegate(p))
        print('connected to %s ' % addr)
        svc = p.getServiceByUUID('00001701-7374-7265-7563-6873656e7365')
        char = svc.getCharacteristics()[0]
        handle = char.valHandle
        p.writeCharacteristic(handle + 1, b'\x01\x00')  # turn on notifications
        p.writeCharacteristic(29, b'\x5a')  # change sampling rate to 90Hz
        self.peripheralInUse.append(p)

    # def connectGlove(self):
    #     # scan for Stretch Sense products for 5 seconds
    #     self.StretchSenseObject.ble_scanning(5)
    #
    #     # list of available peripherals
    #     availablePeri = self.StretchSenseObject.ble_getListPeripheralAvailable()
    #     avail_peris = [peri.addr for peri in availablePeri[1:]]
    #
    #     # get name of known peripherals from yaml file
    #     knownPeripherals = open("/home/husky/borealis_ws/src/stretchsense/src/knownPeripherals.yaml")
    #     kP = yaml.load(knownPeripherals, Loader=yaml.FullLoader)
    #     Gloves = kP['Gloves']
    #
    #     # user selects a glove to connect
    #     if len(avail_peris) > 0:
    #         print('Select a glove to connect\n')
    #         for i in range(len(avail_peris)):
    #             if avail_peris[i] in Gloves.keys():
    #                 print('\t %i. %s' % (i, Gloves[avail_peris[i]]))
    #             else:
    #                 print('\t %i. Unknown, addr: %s' % (i, avail_peris[i]))
    #     else:
    #         print('no gloves found')
    #
    #     selected = int(input('\nSelect from 0 to %i for a glove ' % (len(avail_peris) - 1)))
    #     addr = avail_peris[selected]
    #     print('connecting to addr %s' % addr)
    #     self.StretchSenseObject.ble_connectOnePeripheral(addr)
    #     connected = list(set([c.addr for c in self.StretchSenseObject.listPeripheralIsConnected]))
    #     print('connected to %s ' % connected[0])
    #     return True

    def findModel(self):
        if os.path.isfile(self.thetafile):
            cal = input('Found an old model file. \nCalibrate again? Y/N ')
            if cal in ['Y', 'y']:
                self.haveTheta = False
            else:
                self.haveTheta = True
                TrainingData.CaptureCalibrationData = False
                TrainingData.complete = True
                theta = pd.read_csv(self.thetafile, sep=',',header=None)
                self.mtheta = TrainingData.mtheta = theta.values
        else:
            print('Getting ready to Calibrate in 5 secs')
            time.sleep(5)
            self.haveTheta = False

        return self.haveTheta

    def loadTheta(self):
        self.haveTheta = True
        TrainingData.CaptureCalibrationData = False
        TrainingData.complete = True
        theta = pd.read_csv(self.thetafile, sep=',', header=None)
        self.mtheta = TrainingData.mtheta = theta.values

    def digits(self, position):
        # convert to radians for joint state
        [splay2, thumb, index, middle, ring, pinky] = [p * self.toRad for p in position]
        digits = [splay2, thumb, thumb*0, index, index, index, middle, middle, middle,
                  ring, ring, ring, pinky, pinky, pinky]

        for i in range(0, len(digits)):
            if i in [0, 1]:
                # digits[i] = digits[i] * -1.0
                continue
            if digits[i] > 0:
                digits[i] = 0
            elif digits[i] < -1.57:
                digits[i] = -1.57

        return digits

    def readSensors(self):
        if len(self.peripheralInUse) == 1:
            for p in self.peripheralInUse:
                if p.waitForNotifications(1.0):
                    continue
        return self.capacitance

    def calibrateGlove(self):
        while not self.haveTheta and not rospy.is_shutdown():
            # index of the training segment
            sensorData = self.readSensors()
            if TrainingData.CaptureCalibrationData == True:
                old = time.time()
                index = TrainingData.TrainingIndex
                d = self.digits(self.trainingY[index])
                self.publishCap(calibrate=True, digits = d)
                self.Training.Update(sensorData)
                rospy.loginfo('reading %i' % index)
            elif TrainingData.CaptureCalibrationData == False and TrainingData.complete == False:
                timeLeft = time.time() - old
                d = self.digits(self.trainingY[index+1])
                self.publishCap(calibrate=True, digits = d)
                rospy.loginfo('renewing recording in %f' % timeLeft)
                if timeLeft > 5:
                    TrainingData.CaptureCalibrationData = True
            elif TrainingData.complete == True:
                theta = pd.DataFrame(TrainingData.mtheta)
                theta.to_csv(self.thetafile, index = False, header = False)
                print('saved new model')
                self.haveTheta = True
            self.rate.sleep()

    def publishCap(self, calibrate = False, digits = []):
        if calibrate == False:
            self.loadTheta()
            try:
                while not rospy.is_shutdown():
                    self.Joints.header.seq += 1
                    self.Joints.header.stamp = rospy.Time.now()
                    sens = self.readSensors()
                    sens = np.insert(sens, 0, 1)
                    transformed = self.Solver.ApplyTransformation(sens, self.mtheta)
                    digits = self.digits(transformed)
                    self.Joints.position = digits
                    self.pubjs.publish(self.Joints)
                    self.rate.sleep()
            except:
                print('disconnecting...')
                for p in self.peripheralInUse:
                    p.disconnect()
                quit()
                pass
        else:
            self.Joints.header.seq += 1
            self.Joints.header.stamp = rospy.Time.now()
            self.Joints.position = digits
            self.pubjs.publish(self.Joints)







if __name__ == "__main__":
    # initiate node
    rospy.init_node('stretchsenseCAP', anonymous=True)

    # initialize smart glove
    SmartGlove = SmartGloveSS()
    if SmartGlove.connectGlove():
        if SmartGlove.findModel():
            # Keep the old calibration data
            SmartGlove.publishCap()
        else:
            # time to recalibrate
            SmartGlove.calibrateGlove()
            SmartGlove.publishCap()







