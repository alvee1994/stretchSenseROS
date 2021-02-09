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
import os, subprocess, sys

# ROS
import rospy
from collections import deque
from sensor_msgs.msg import JointState
from stretchsense.msg import ssCap

package_directory = subprocess.check_output(["rospack", "find", "stretchsense"])
package_directory = package_directory.decode().strip('\n')

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
    thetafile = package_directory + "/src/theta_new.csv"

    """
    More Variables
    """
    toRad = 0.01745329251
    fLen = 20
    fingers = deque(maxlen=fLen)
    sumFingers = []
    binFingers = [0,0,0,0,0] # fingers in binary
    change = deque(maxlen=10)
    """
    the initiated peripheral itself
    """
    peripheralInUse = deque(maxlen=1)
    capacitance = []

    """
    Glove (left ot right)
    """
    hand = 'left'
    # handedness = {'left': 1, 'right': -1}

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
        self.Joints.name = ["metacarpal_thumb_splay_2", "thumb_meta_prox",
                            "thumb_prox_inter", "metacarpal_index", "index_prox_inter", "index_inter_dist",
                            "metacarpal_middle", "middle_prox_inter", "middle_inter_dist",
                            "metacarpal_ring",
                            "ring_prox_inter", "ring_inter_dist", "metacarpal_pinky", "pinky_prox_inter",
                            "pinky_inter_dist"]

    def connectGlove(self, calibrate=True):
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
        knownPeripherals = open(package_directory + "/src/knownPeripherals.yaml")
        kP = yaml.load(knownPeripherals, Loader=yaml.FullLoader)
        Gloves = kP['Gloves'] # all the addresses
        service = kP['Service'] # BLEservice

        # user selects a glove to connect
        if len(listOfPeripheralsAvailable) > 0 and calibrate:
            print('Select a stretchsense glove to connect\n')
            for i in range(len(listOfPeripheralsAvailable)):
                if listOfPeripheralsAvailable[i] in Gloves.keys():
                    print('\t %i. %s' % (i, Gloves[listOfPeripheralsAvailable[i]][0]))
                else:
                    print('\t %i. Unknown, addr: %s' % (i, listOfPeripheralsAvailable[i]))

            selected = int(input('\nSelect from 0 to %i for a glove ' % (len(listOfPeripheralsAvailable) - 1)))
            addr = listOfPeripheralsAvailable[selected]

            """
            Left or Right hand
            """
            if addr in Gloves.keys():
                self.hand = Gloves[addr][1]
            else:
                self.hand = input('Is this a left handed or right handed glove?\n\t(left/right): ')

            rospy.set_param('/hand', self.hand)
            print('connecting to addr %s' % addr)
            self.connectOnePeripheral(addr, service)
            return True
        elif len(listOfPeripheralsAvailable) > 0 and not calibrate:
            addr = listOfPeripheralsAvailable[0]
            self.hand = Gloves[addr][1]
            rospy.set_param('/hand', self.hand)
            print('connecting to addr %s' % addr)
            self.connectOnePeripheral(addr, service)
            return True
        else:
            print(' No gloves found.\n')



    def connectOnePeripheral(self, addr, service):
        p = btle.Peripheral(addr, 'random')
        p.withDelegate(StretchSenseDelegate(p))
        print('connected to %s ' % addr)
        svc = p.getServiceByUUID(service)
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

    def findModel(self, calibrate=True):
        if os.path.isfile(self.thetafile) and calibrate:
            cal = input('Found an old model file. \nCalibrate again? Y/N ')
            if cal in ['Y', 'y']:
                self.haveTheta = False
            else:
                self.haveTheta = True
                TrainingData.CaptureCalibrationData = False
                TrainingData.complete = True
                theta = pd.read_csv(self.thetafile, sep=',',header=None)
                self.mtheta = TrainingData.mtheta = theta.values
        elif not calibrate:
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
        [splay2, thumb, index, middle, ring, pinky] = np.clip([p * self.toRad for p in position], -1.57, 0)



        digits = np.array([splay2, thumb, thumb * 0, index, index, index, middle, middle, middle,
                               ring, ring, ring, pinky, pinky, pinky])

        if self.hand == 'right':
            print('right hand')
            digits = -1 * np.flip(digits)

        """
        The following block creates deque with mean values for individual fingers.
        The size of the deque is declared in the constructor. 
        """
        newest = [thumb, index, middle, ring, pinky]
        self.fingers.append(newest)
        if len(self.fingers) == self.fLen:
            oldest = self.fingers[0]
            self.sumFingers = self.sumFingers - oldest + newest
        else:
            self.sumFingers = np.sum(self.fingers, axis=0) / len(self.fingers)

        older = self.binFingers
        self.binFingers = np.where(self.sumFingers > -1.0, 1, 0)
        delta = older - self.binFingers
        if np.any(delta):
            self.change.append(delta)
        """
        End of the finger block
        """

        # print(self.change)

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
                d = self.digits(self.trainingY[index+1]) # ignore fingers when calibrating
                self.publishCap(calibrate=True, digits = d)
                rospy.loginfo('renewing recording in %f' % timeLeft)
                if timeLeft > 5:
                    TrainingData.CaptureCalibrationData = True
            elif TrainingData.complete == True:
                theta = pd.DataFrame(TrainingData.mtheta)
                filename = '/src/' + input('What should I name the new file (without extension)? ') + '.csv'
                theta.to_csv(filename, index = False, header = False)
                self.thetafile = filename
                print('saved new model as %s' % filename)
                self.haveTheta = True
            self.rate.sleep()

    def prepHeader(self):
        self.Joints.header.seq += 1
        self.Joints.header.stamp = self.Position.header.stamp = rospy.Time.now()


    def publishCap(self, calibrate = False, digits = []):
        if calibrate == False:
            self.loadTheta()
            try:
                while not rospy.is_shutdown():
                    self.prepHeader()
                    sens = self.readSensors()
                    sens = np.insert(sens, 0, 1)
                    transformed = self.Solver.ApplyTransformation(sens, self.mtheta)
                    digits = self.digits(transformed)
                    self.Joints.position = digits
                    self.Position.values = self.binFingers
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
            self.prepHeader()
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







if __name__ == "__main__":
    # initiate node
    rospy.init_node('stretchsenseCAP', anonymous=True)

    # initialize smart glove
    SmartGlove = SmartGloveSS()
    calibrate = True if sys.argv[1] == 'True' else False
    if SmartGlove.connectGlove(calibrate=calibrate):
        if SmartGlove.findModel(calibrate=calibrate):
            # Keep the old calibration data
            SmartGlove.publishCap()
        else:
            # time to recalibrate
            SmartGlove.calibrateGlove()
            SmartGlove.publishCap()







