#!/usr/bin/env python3

import time
import numpy as np
import yaml
import pandas as pd
from bluepy import btle
import binascii
import os.path
from lib import TrainingData, SolveLeastSquares
import os

# ROS
import rospy, rospkg
from collections import deque
from sensor_msgs.msg import JointState
from stretchsense.msg import ssCap

rospack = rospkg.RosPack()


class StretchSenseDelegate(btle.DefaultDelegate):
    def __init__(self, params):
        btle.DefaultDelegate.__init__(self)
        self.val = [0, 1, 2, 3]

    def handleNotification(self, cHandle, data):
        decimalValue = (binascii.b2a_hex(data))
        splitted = [decimalValue[i:i + 4] for i in range(0, len(decimalValue), 4)]
        self.val = np.array(list(map(lambda x: int((x), 16) / 10, splitted)))
        idx = np.nonzero(self.val)
        self.val = self.val[idx]
        SmartGloveSS.capacitance = self.val


class SmartGloveSS:
    """
    Variables to check old calibration data
    """
    haveTheta = False
    package_directory = rospack.get_path('stretchsense')
    thetafile = package_directory + "/src/data/sam_theta.csv"
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
        self.pubfingers = rospy.Publisher('/fingers', ssCap, queue_size=2)
        self.Joints = JointState()
        self.Position = ssCap()
        self.rate = rospy.Rate(200)
        self.mtheta = []
        # get training data
        self.trainingY = self.Training.getTrainingData()

        self.Joints.header.frame_id = "ssFingers"
        self.Joints.name = ["left_" + string for string in ["metacarpal_thumb_splay_2", "thumb_meta_prox",
                                                            "thumb_prox_inter", "metacarpal_index", "index_prox_inter",
                                                            "index_inter_dist", "metacarpal_middle",
                                                            "middle_prox_inter",
                                                            "middle_inter_dist", "metacarpal_ring",
                                                            "ring_prox_inter", "ring_inter_dist", "metacarpal_pinky",
                                                            "pinky_prox_inter", "pinky_inter_dist"]]

    def connectGlove(self):
        # initiate scanner to scan and filter stretchsense devices with the following parameters
        params = {'hci': 0,
                  'timeout': 4,
                  'sensitivity': -128,
                  'discover': True,
                  'all': True,
                  'new': True,
                  'verbose': True}

        scanner = btle.Scanner().withDelegate(StretchSenseDelegate(params))
        devices = scanner.scan(3)
        listOfPeripheralsAvailable = []

        for dev in devices:
            for (sdid, desc, val) in dev.getScanData():
                if val == 'StretchSense':
                    listOfPeripheralsAvailable.append(dev.addr)

        # get name of known peripherals from yaml file
        knownPeripherals = open(self.package_directory + "/src/data/knownPeripherals.yaml")
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

            selected = int(input('\nSelect from 0 to %i for a glove ' % (len(listOfPeripheralsAvailable) - 1)))
            addr = listOfPeripheralsAvailable[selected]
            print('connecting to addr %s' % addr)
            self.connectOnePeripheral(addr)
            return True

        else:
            print(' No gloves found.\n')

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

    def findModel(self):
        if os.path.isfile(self.thetafile):
            cal = input('Found an old model file. \nCalibrate again? Y/N ')
            if cal in ['Y', 'y']:
                self.haveTheta = False
            else:
                self.haveTheta = True
                TrainingData.CaptureCalibrationData = False
                TrainingData.complete = True
                theta = pd.read_csv(self.thetafile, sep=',', header=None)
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
        digits = [splay2, thumb, thumb * 0, index, index, index, middle, middle, middle,
                  ring, ring, ring, pinky, pinky, pinky]
        fingers = [thumb, index, middle, ring, pinky]

        for i in range(0, len(digits)):
            if i in [0, 1]:
                # digits[i] = digits[i] * -1.0
                continue
            if digits[i] > 0:
                digits[i] = 0
            elif digits[i] < -1.57:
                digits[i] = -1.57

        return digits, fingers

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
                d, _ = self.digits(self.trainingY[index])
                self.publishCap(calibrate=True, digits=d)
                self.Training.Update(sensorData)
                rospy.loginfo('reading %i' % index)
            elif TrainingData.CaptureCalibrationData == False and TrainingData.complete == False:
                timeLeft = time.time() - old
                d, _ = self.digits(self.trainingY[index + 1])  # ignore fingers when calibrating
                self.publishCap(calibrate=True, digits=d)
                rospy.loginfo('renewing recording in %f' % timeLeft)
                if timeLeft > 5:
                    TrainingData.CaptureCalibrationData = True
            elif TrainingData.complete == True:
                theta = pd.DataFrame(TrainingData.mtheta)
                newfile = self.package_directory + "/src/data/theta_" + str(rospy.Time.now()) + ".csv"
                self.thetafile = newfile
                theta.to_csv(self.thetafile, index=False, header=False)
                print('saved new model')
                self.haveTheta = True
            self.rate.sleep()

    def publishCap(self, calibrate=False, digits=[]):
        if calibrate == False:
            self.loadTheta()
            try:
                while not rospy.is_shutdown():
                    self.Joints.header.seq += 1
                    self.Joints.header.stamp = rospy.Time.now()
                    sens = self.readSensors()
                    sens = np.insert(sens, 0, 1)
                    transformed = self.Solver.ApplyTransformation(sens, self.mtheta)
                    digits, fingers = self.digits(transformed)
                    self.Joints.position = digits
                    self.Position.values = fingers
                    self.pubjs.publish(self.Joints)
                    self.pubfingers.publish(self.Position)
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

    # def quitProcess(self):
    #     print('ending application\n')
    #     try:
    #         for p in self.peripheralInUse:
    #             p.disconnect()
    #     except:
    #         print('no gloves were connected\n')
    #     quit()


"""
Procedure:
1. Find all gloves
2. Connect to found glove
3. Data:
        a. train if data doesnt exist for user
        b. use existing data
4. 
"""

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
