#!/usr/bin/env python3
"""Main application script."""
import time
from typing import List, Tuple
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
    """Encapsulates a StretchSense Glove object.

    Handles connecting to the glove via Bluetooth Low Energy, publishing to
    ROS, and calibration of the glove. (Which is obviously suboptimal design)

    TODO: split this class up into smaller classes.
    
    Attributes:
        has_theta:
            A bool representing whether theta values are available
        PACKAGE_DIRECTORY:
            A str containing the file path to this package within the ROS
            workspace
        thetafile:
            The file path to the theta file. Will be the path to theta_default
            by default.

        TO_RAD:
            A constant used to convert angles from degrees to radians by
            multiplication.
        SERVICE_UUID:
            A string containing the service uuid for the stretchsense gloves 
        peripheral_in_use:
            The peripheral currently being used in the application.

        Solver:
            The solver used to transform capacitance data into angle data.
        TrainingData:
            The data used to train the model and obtain the theta values.
        JointPublisher:
            The publisher used to publish joint data to ROS.
        FingerPublisher:
            The publisher used to publish finger data to ROS.
        Joints:
            A container for the joint data to be published.
        Fingers:
            A container for the finger data to be published.
        Rate:
            An instance of rospy.Rate used to create delays for user input
        capacitance:
            A numpy array used to store the capacitance data from the
            gloves' sensors.
        training_targets:
            The target angle data for each training gesture inside
            TrainingData.
    """
    # Variables to check old calibration data
    has_theta = False
    PACKAGE_DIRECTORY = rospack.get_path('stretchsense')
    thetafile = PACKAGE_DIRECTORY + "/src/data/theta_default.csv"
    
    # More Variables
    TO_RAD = 0.01745329251
    SERVICE_UUID = '00001701-7374-7265-7563-6873656e7365'

    # the initiated peripheral itself
    peripheral_in_use = deque(maxlen=1)

    def __init__(self):
        """Constructor for the Smart Glove class."""

        # machine learning
        self.Solver = SolveLeastSquares.SolveLeastSquares()
        self.TrainingData = TrainingData.TrainingData()

        # stretchsense API
        self.JointPublisher = rospy.Publisher('/joint_states', JointState, queue_size=2)
        self.FingerPublisher = rospy.Publisher('/Fingers', ssCap, queue_size=2)
        self.Joints = JointState()
        self.Fingers = ssCap()
        self.Rate = rospy.Rate(200)
        self.capacitance: np.ndarray

        # get training data
        self.training_targets = self.TrainingData.get_training_data()

        self.Joints.header.frame_id = "ssFingers"
        _joint_list = ["metacarpal_thumb_splay_2", "thumb_meta_prox",
                       "thumb_prox_inter", "metacarpal_index", "index_prox_inter",
                       "index_inter_dist", "metacarpal_middle",
                       "middle_prox_inter",
                       "middle_inter_dist", "metacarpal_ring",
                       "ring_prox_inter", "ring_inter_dist", "metacarpal_pinky",
                       "pinky_prox_inter", "pinky_inter_dist"]
        self.Joints.name = ["left_" + string for string in _joint_list]

    def connect_glove(self) -> bool:
        """Connect to a selected stretchsense glove.
        
        Scans for available Bluetooth Low Energy devices, allows user to
        choose a stretchsense glove to connect to, then connects to the
        chosen glove.
        """

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

    def set_up_glove(self, addr: str) -> None:
        """Sets up the glove at the specified address for data collection.
        
        Initialises a bluepy.btle.Peripheral object for the given address
        with a StretchSenseDelegate object. Then turns on notifications and
        changes the sampling rate. Finally, updates peripheral_in_use.
        """

        Glove = btle.Peripheral(addr, 'random')
        Glove.withDelegate(StretchSenseDelegate())
        print('connected to %s ' % addr)
        svc = Glove.getServiceByUUID(SmartGloveSS.SERVICE_UUID)
        char = svc.getCharacteristics()[0]
        handle = char.valHandle
        Glove.writeCharacteristic(handle + 1, b'\x01\x00')  # turn on notifications
        Glove.writeCharacteristic(29, b'\x5a')  # change sampling rate to 90Hz
        self.peripheral_in_use.append(Glove)

    def find_model(self) -> bool:
        """Checks if a default theta file exists.

        Checks if the file path in self.thetafile has a valid file. If it 
        does not, set the has_theta flag to begin calibration. If it does,
        prompt user to decide whether to recalibrate glove. If yes, set the
        has_theta flag to begin calibration. If no, retrieve the theta values
        from the default theta file and update mtheta.

        Returns:
            False if calibration is required, and
            True otherwise.
        """
        if os.path.isfile(self.thetafile):
            cal = input('Found an old model file. \nCalibrate again? Y/N ')
            if cal in ['Y', 'y', 'yes', 'Yes']:
                self.has_theta = False
            else:
                self.has_theta = True
                self.TrainingData.is_calibrating = False
                self.TrainingData.is_complete = True
                theta = pd.read_csv(self.thetafile, sep=',', header=None)
                self.TrainingData.mtheta = theta.values
        else:
            print('Getting ready to Calibrate in 5 secs')
            time.sleep(5)
            self.has_theta = False

        return self.has_theta

    def load_theta(self) -> None:
        """Loads data from the default file to mtheta.

        Sets flags, then retrieves the theta values from the default theta file
        to update mtheta.
        """

        self.has_theta = True
        self.TrainingData.is_calibrating = False
        self.TrainingData.is_complete = True
        theta = pd.read_csv(self.thetafile, sep=',', header=None)
        self.TrainingData.mtheta = theta.values

    def process_angles(self,
                       angle_data: np.ndarray) -> Tuple[List, List]:
        """Reformats the angle data in degrees for publishing to ROS.

        Takes in a vector containing the angle of the 6 measured joints in
        degrees and reformats it into vectors that can be published.

        Args:
            joint_angles:
                A numpy array containing the angle of the 6 measured joints in
                degrees
        
        Returns:
            A tuple containing 2 lists, one containing data to be published
            by JointPublisher, and another containing data to be published
            by FingerPublisher, where the data is in radians.
        """

        # convert to radians for joint state
        angles_in_rad = [angle * self.TO_RAD for angle in angle_data]
        [splay2, thumb, index, middle, ring, pinky] = angles_in_rad
        joints = [splay2, thumb, 0, index, index, index, middle, middle, 
                  middle, ring, ring, ring, pinky, pinky, pinky]
        fingers = [thumb, index, middle, ring, pinky]

        for i in range(len(joints)):
            if joints[i] > 0:
                joints[i] = 0
            elif joints[i] < -1.57:
                joints[i] = -1.57

        return joints, fingers

    def read_sensors(self) -> np.ndarray:
        """Gets a sample of capacitance data from the glove.

        Waits for the glove to send a sample of capacitance data with a timeout
        of 1 second. Then retrieves the capacitatance data from
        self.capacitance and returns it if it is a valid data set.

        Returns:
            A numpy array with n capacitance readings where n = the number of
            sensors on the glove.
        
        Raises:
            InvalidCapacitanceError: The data stored inside self.capacitance
            does not have the required number of readings.
        """

        if len(self.peripheral_in_use) == 1:
            for p in self.peripheral_in_use:
                if p.waitForNotifications(1.0):
                    continue

        if len(self.capacitance) == self.TrainingData.NUM_SENSORS:
            return self.capacitance
        else:
            raise InvalidCapacitanceError(self.capacitance)

    def calibrate_glove(self) -> None:
        """Calibrates the glove.

        Gets sensor data, trains a linear regression model, then creates
        and saves a new theta file.
        """

        while not self.has_theta and not rospy.is_shutdown():
            # index of the training segment
            try:
                sensor_data = self.read_sensors()
                if self.TrainingData.is_calibrating == True: 
                    old = time.time()
                    index = self.TrainingData.gesture_index
                    d, _ = self.process_angles(self.training_targets[index])
                    self.publish_capacitance(calibrate=True, joints=d)
                    self.TrainingData.update_sample(sensor_data)
                    rospy.loginfo(f"reading: {index}")

                elif (self.TrainingData.is_calibrating == False and
                      self.TrainingData.is_complete == False):
                    time_left = time.time() - old
                    # ignore fingers when calibrating
                    d, _ = self.process_angles(self.training_targets[index+1])
                    self.publish_capacitance(calibrate=True, joints=d)
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

    def publish_capacitance(self, calibrate=False, joints=[]) -> None:
        """Publishes data with Joint and Finger Publisher.

        If not calibrating, takes in capacitance data from the sensors, convert
        it into joint angle data, then reformat it to be published by
        JointPublisher and FingerPublisher. If calibrating, publish the given
        Joint data.

        TODO: split this into 2 methods

        Args:
            calibrate:
                Boolean that represents whether the glove is currently being
                calibrated.
            joints:
                Optional parameter that may contain joint data to be published.
                
        """
        if calibrate == False:
            self.load_theta()
            while not rospy.is_shutdown():
                try:
                    self.Joints.header.seq += 1
                    self.Joints.header.stamp = rospy.Time.now()
                    sensor_data = self.read_sensors()
                    sensor_data = np.insert(sensor_data, 0, 1)
                    transformed = self.Solver.apply_transformation(sensor_data,
                        self.TrainingData.mtheta)
                    joints, fingers = self.process_angles(transformed)
                    self.Joints.position = joints
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
            self.Joints.position = joints
            self.JointPublisher.publish(self.Joints)


def main():
    """The main process for the Smart Glove Application.

    Finds all gloves, connects to a selected glove, train if necessary,
    then publish the angle data to ROS.
    """

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
