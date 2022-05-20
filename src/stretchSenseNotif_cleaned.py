#!/usr/bin/env python3

import time
from typing import List
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
    """Handles notifications from the glove
    
    This class handles the notifications sent from the glove via Bluetooth
    Low Energy.
    """

    def __init__(self):
        """Constructor for StretchSenseDelegate"""

        super().__init__()

    def handleNotification(self, cHandle, data) -> None:
        """Implementation of the handleNotification method in DefaultDelegate.
        
        Takes in bytestring data from the glove via BLE and converts it into
        capacitatance data in the form of a numpy array vector of type int.

        Args:
            cHandle:
                Unused parameter.
            data:
                Bytestring data from glove's sensors.
        """

        hex_vals = (binascii.b2a_hex(data))
        split_vals = np.array([int(hex_vals[i:i + 4], 16) / 10 
                               for i in range(0, len(hex_vals), 4)])
        # val = np.array(list(map(lambda x: int((x), 16) / 10, splitted)))
        capacitance = split_vals[split_vals != 0]
        SmartGloveSS.capacitance = capacitance

class InvalidCapacitanceError(Exception):
    """Exception that is raised when the capacitance data is invalid.

    Raised when the capacitance data received cannot be used for analysis
    and/or training.
    """

    def __init__(self, capacitance: np.ndarray):
        """ Constructor for the exception.

        Message contains a string containing the erroneous capacitance data.

        Args:
            capacitance: numpy array containing a sample of capacitance data.
        """

        super().__init__(f"{capacitance} is an invalid capacitance data entry")

class SmartGloveSS:
    """
    Variables to check old calibration data
    """
    has_theta = False
    PACKAGE_DIRECTORY = rospack.get_path('stretchsense')
    thetafile = PACKAGE_DIRECTORY + "/src/data/theta_default.csv"
    """
    More Variables
    """
    TO_RAD = 0.01745329251
    SERVICE_UUID = '00001701-7374-7265-7563-6873656e7365'

    """
    the initiated peripheral itself
    """
    peripheral_in_use = deque(maxlen=1)

    def __init__(self):

        # machine learning
        self.Solver = SolveLeastSquares.SolveLeastSquares()
        self.TrainingData = TrainingData.TrainingData()

        # stretchsense API
        self.JointPublisher = rospy.Publisher('/joint_states', JointState, queue_size=2)
        self.FingerPublisher = rospy.Publisher('/Fingers', ssCap, queue_size=2)
        self.Joints = JointState()
        self.Fingers = ssCap()
        self.Rate = rospy.Rate(200)
        self.mtheta = []
        self.capacitance: np.ndarray

        # get training data
        self.training_targets = self.TrainingData.get_training_data()

        self.Joints.header.frame_id = "ssFingers"
        joint_list = ["metacarpal_thumb_splay_2", "thumb_meta_prox",
                       "thumb_prox_inter", "metacarpal_index", "index_prox_inter",
                       "index_inter_dist", "metacarpal_middle",
                       "middle_prox_inter",
                       "middle_inter_dist", "metacarpal_ring",
                       "ring_prox_inter", "ring_inter_dist", "metacarpal_pinky",
                       "pinky_prox_inter", "pinky_inter_dist"]
        self.Joints.name = ["left_" + string for string in joint_list]

    def connect_glove(self):

        scanner = btle.Scanner().withDelegate(StretchSenseDelegate())
        devices = scanner.scan(3)
        available_peripherals = []

        for dev in devices:
            for (sdid, desc, val) in dev.getScanData():
                if val == 'StretchSense':
                    available_peripherals.append(dev.addr)

        # get name of known peripherals from yaml file
        known_peripherals_yaml = open(self.PACKAGE_DIRECTORY + 
                                      "/src/data/knownPeripherals.yaml")
        known_peripherals = yaml.load(known_peripherals_yaml,
                                      Loader=yaml.FullLoader)
        known_gloves = known_peripherals['Gloves']

        # user selects a glove to connect
        if len(available_peripherals) > 0:
            print('Select a glove to connect\n')
            for idx, addr in enumerate(available_peripherals):
                if addr in known_gloves.keys():
                    print(f"{idx}. {addr}")
                else:
                    print(f"{idx}. Unknown, addr: {addr}")

            selected = int(input("\n Select glove from 0 to " +
                                 f"{len(available_peripherals) - 1}: "))
            addr = available_peripherals[selected]
            print(f"\n connecting to addr: {addr}")
            self.set_up_glove(addr)
            return True

        else:
            print(' No gloves found.\n')

    def set_up_glove(self, addr):
        Glove = btle.Peripheral(addr, 'random')
        Glove.withDelegate(StretchSenseDelegate())
        print('connected to %s ' % addr)
        svc = Glove.getServiceByUUID(SmartGloveSS.SERVICE_UUID)
        char = svc.getCharacteristics()[0]
        handle = char.valHandle
        Glove.writeCharacteristic(handle + 1, b'\x01\x00')  # turn on notifications
        Glove.writeCharacteristic(29, b'\x5a')  # change sampling rate to 90Hz
        self.peripheral_in_use.append(Glove)

    def find_model(self):
        if os.path.isfile(self.thetafile):
            cal = input('Found an old model file. \nCalibrate again? Y/N ')
            if cal in ['Y', 'y', 'yes', 'Yes']:
                self.has_theta = False
            else:
                self.has_theta = True
                self.TrainingData.is_calibrating = False
                self.TrainingData.is_complete = True
                theta = pd.read_csv(self.thetafile, sep=',', header=None)
                self.mtheta = self.TrainingData.mtheta = theta.values
        else:
            print('Getting ready to Calibrate in 5 secs')
            time.sleep(5)
            self.has_theta = False

        return self.has_theta

    def load_theta(self):
        self.has_theta = True
        self.TrainingData.is_calibrating = False
        self.TrainingData.is_complete = True
        theta = pd.read_csv(self.thetafile, sep=',', header=None)
        self.mtheta = self.TrainingData.mtheta = theta.values

    def process_angles(self, joint_angles: np.ndarray):
        # convert to radians for joint state
        angles_in_rad = [angle * self.TO_RAD for angle in joint_angles]
        [splay2, thumb, index, middle, ring, pinky] = angles_in_rad
        digits = [splay2, thumb, 0, index, index, index, middle, middle, 
                  middle, ring, ring, ring, pinky, pinky, pinky]
        fingers = [thumb, index, middle, ring, pinky]

        for i in range(len(digits)):
            if digits[i] > 0:
                digits[i] = 0
            elif digits[i] < -1.57:
                digits[i] = -1.57

        return digits, fingers

    def read_sensors(self):
        if len(self.peripheral_in_use) == 1:
            for p in self.peripheral_in_use:
                if p.waitForNotifications(1.0):
                    continue

        if len(self.capacitance) == self.TrainingData.NUM_SENSORS:
            return self.capacitance
        else:
            raise InvalidCapacitanceError(self.capacitance)

    def calibrate_glove(self):
        while not self.has_theta and not rospy.is_shutdown():
            # index of the training segment
            try:
                sensorData = self.read_sensors()
                if self.TrainingData.is_calibrating == True: 
                    old = time.time()
                    index = self.TrainingData.gesture_index
                    d, _ = self.process_angles(self.training_targets[index])
                    self.publish_capacitance(calibrate=True, digits=d)
                    self.TrainingData.update_sample(sensorData)
                    rospy.loginfo(f"reading: {index}")

                elif (self.TrainingData.is_calibrating == False and
                      self.TrainingData.is_complete == False):
                    time_left = time.time() - old
                    # ignore fingers when calibrating
                    d, _ = self.process_angles(self.training_targets[index+1])
                    self.publish_capacitance(calibrate=True, digits=d)
                    rospy.loginfo(f"renewing recording in {time_left}")
                    if time_left > 5:
                        self.TrainingData.is_calibrating = True

                elif self.TrainingData.is_complete == True:
                    theta = pd.DataFrame(self.TrainingData.mtheta)
                    self.thetafile = (self.PACKAGE_DIRECTORY
                                      + "/src/data/theta_"
                                      + str(rospy.Time.now())
                                      + ".csv")
                    theta.to_csv(self.thetafile, index=False, header=False)
                    print('saved new model')
                    self.has_theta = True

                self.Rate.sleep()
            except InvalidCapacitanceError as e:
                print(e)
                print("skipping...")
                continue

    def publish_capacitance(self, calibrate=False, digits=[]):
        if calibrate == False:
            self.load_theta()
            while not rospy.is_shutdown():
                try:
                    self.Joints.header.seq += 1
                    self.Joints.header.stamp = rospy.Time.now()
                    sensor_data = self.read_sensors()
                    sensor_data = np.insert(sensor_data, 0, 1)
                    transformed = self.Solver.apply_transformation(sensor_data,
                                                                   self.mtheta)
                    digits, fingers = self.process_angles(transformed)
                    self.Joints.position = digits
                    self.Fingers.values = fingers
                    self.JointPublisher.publish(self.Joints)
                    self.FingerPublisher.publish(self.Fingers)
                    self.Rate.sleep()
                except InvalidCapacitanceError as e:
                    print(e)
                    print('skipping...')
                    continue
                    
        else:
            self.Joints.header.seq += 1
            self.Joints.header.stamp = rospy.Time.now()
            self.Joints.position = digits
            self.JointPublisher.publish(self.Joints)


"""
Procedure:
1. Find all gloves
2. Connect to found glove
3. Data:
        a. train if data doesnt exist for user
        b. use existing data
4. 
"""
def main():
    # initiate node
    rospy.init_node('stretchsenseCAP', anonymous=True)

    # initialize smart glove
    SmartGlove = SmartGloveSS()
    if SmartGlove.connect_glove():
        if SmartGlove.find_model():
            # Keep the old calibration data
            SmartGlove.publish_capacitance()
        else:
            # time to recalibrate
            SmartGlove.calibrate_glove()
            SmartGlove.publish_capacitance()


if __name__ == "__main__":
    main()
