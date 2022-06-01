#!/usr/bin/python3
"""The main application script."""

import time
from typing import List, Tuple
import numpy as np
import math

import rospy, rospkg
from sensor_msgs.msg import JointState
from stretchsense.msg import ssCap

from peripheral import bluetooth_handler, stretchsense_peripheral
from model import model, trainer
from user import user

rospack = rospkg.RosPack()

class ROSHandler:
    """This class handles publishing data to ROS."""

    _PACKAGE_DIRECTORY = rospack.get_path('stretchsense')
    _TO_RAD = math.pi / 180

    def __init__(self):
        # The current user 
        self._user: user.User

        # For connecting to peripheral
        self._bluetooth_handler = bluetooth_handler.BluetoothHandler(
            ROSHandler._PACKAGE_DIRECTORY)
        self._glove: stretchsense_peripheral.StretchSensePeripheral

        # For machine learning
        self._model: model.Model
        self._trainer: trainer.Trainer

        # For publishing to ROS
        self._joint_publisher = rospy.Publisher('/joint_states',
                                                JointState,
                                                queue_size=2)
        self._finger_publisher = rospy.Publisher('/fingers',
                                                 ssCap,
                                                 queue_size=2)
        self._joint_states = JointState()
        self._finger_states = ssCap()
        self._rate = rospy.Rate(200)
        self._joint_states.header.frame_id = "ssFingers"
        self._joint_states.name = [
            "metacarpal_thumb_splay_2", "thumb_meta_prox",
            "thumb_prox_inter", "metacarpal_index", "index_prox_inter",
            "index_inter_dist", "metacarpal_middle",
            "middle_prox_inter",
            "middle_inter_dist", "metacarpal_ring",
            "ring_prox_inter", "ring_inter_dist", "metacarpal_pinky",
            "pinky_prox_inter", "pinky_inter_dist"
        ]

    def _register_user(self) -> user.User:
        """Gets the current user.
        
        Prompts user for username and returns a User object with the given
        username.

        Returns:
            User object where its name attribute is the given username.
        """

        username = input("Enter username: ")
        return user.User(username)


    def connect(self) -> bool:
        """Connect to peripheral and set up publisher.
        
        Returns:
            True if peripheral can be connected to.
            False otherwise
        """

        # Get the user
        self._user = self._register_user()
        
        # Connect to a peripheral
        glove = self._bluetooth_handler.connect_peripheral()

        if glove: # If a glove is found
            # Set up parameters
            self._joint_states.name = [glove.SIDE
                + "_"
                + name for name in self._joint_states.name]
            self._glove = glove
            self._model = model.Model(self._glove,
                                      self._user,
                                      self._PACKAGE_DIRECTORY)
            self._trainer = trainer.Trainer(self._model)

            return True
        else: # If no glove found
            return False

    def requires_calibration(self) -> bool:
        """Check if the peripheral needs to be calibrated.
        
        Attempts to load theta values in self._model, if no values are found,
        or user chooses to calibrate again, return True. Else return False.

        Returns:
            True if calibration is required.
            False otherwise.
        """

        return not self._model.find_model()

    def _process_angles(self,
                       angle_data: np.ndarray) -> Tuple[List, List]:
        """Reformats the angle data in degrees for publishing to ROS.

        Takes in a vector containing the angle of the 6 measured joints in
        degrees and reformats it into vectors that can be published.

        Args:
            angle_data:
                A numpy array containing the angle of the 6 measured joints in
                degrees
        
        Returns:
            A tuple containing 2 lists, one containing data to be published
            by _joint_publisher, and another containing data to be published
            by _finger_publisher, where the data is in radians.
        """

        # Convert input to radians
        angles_in_rad = [angle * self._TO_RAD for angle in angle_data]
        [splay2, thumb, index, middle, ring, pinky] = angles_in_rad

        # Get values to be published to joint_publisher
        joints = [splay2, thumb, 0, index, index, index, middle, middle, 
                  middle, ring, ring, ring, pinky, pinky, pinky]

        # Get values to be published to finger_publisher
        fingers = [thumb, index, middle, ring, pinky]

        # Trimming the values to be between -pi/2 and 0
        for i in range(len(joints)):
            if joints[i] > 0:
                joints[i] = 0
            elif joints[i] < -1.57:
                joints[i] = -1.57

        # Return the lists as a tuple
        return joints, fingers
        
    def _publish_target(self, joints: np.ndarray) -> None:
        """Publishes target angles during calibration for user to follow.
        
        Takes in a numpy array of 15 joint angles and publishes the joint
        data to ROS.
        
        Args:
            joints:
                A numpy array representing each joint angle in radians.
        """

        self._joint_states.header.seq += 1
        self._joint_states.header.stamp = rospy.Time.now()
        self._joint_states.position = joints
        self._joint_publisher.publish(self._joint_states)

    def publish_input(self) -> None:
        """Publishes the user's input.
        
        Reads sensor data, processes it, then publishes
        the joint and finger data.
        """

        while not rospy.is_shutdown():
            self._joint_states.header.seq += 1
            self._joint_states.header.stamp = rospy.Time.now()

            # Read data from sensors
            sensor_data = self._glove.read_sensors()
            if sensor_data is None:
                # If sensor data was invalid and returned None, skip
                continue

            # Transform capacitance data to get angle data using the model
            sensor_data = np.insert(sensor_data, 0, 1)
            transformed = self._model.apply_transformation(sensor_data)

            # Get the values to be published
            joints, fingers = self._process_angles(transformed)

            # Publish above values
            self._joint_states.position = joints
            self._finger_states.values = fingers
            self._joint_publisher.publish(self._joint_states)
            self._finger_publisher.publish(self._finger_states)
            self._rate.sleep()

    def calibrate(self) -> None:
        """Calibrates the glove.

        Gets sensor data, trains a linear regression model, then creates
        and saves a new theta file.
        """
        while not rospy.is_shutdown():
            # Read sensor data
            sensor_data = self._glove.read_sensors()
            if sensor_data is None:
                # If sensor data was invalid and returned None, skip
                continue
            
            if self._trainer.is_calibrating:
                    # If data for a particular gesture is being collected

                    # Get time
                    old = time.time()

                    # Publish the targets
                    index = self._trainer.gesture_index
                    joints, _ = self._process_angles(
                        self._trainer.TRAINING_TARGETS[index])
                    self._publish_target(joints)

                    # Update the trainer with the sample sensor data
                    self._trainer.update_sample(sensor_data)
                    rospy.loginfo(f"reading: {index}")

            elif (not self._trainer.is_calibrating and
                  not self._trainer.is_complete):
                # If sufficient data was collected for a gesture

                # Get remaining time
                time_left = time.time() - old

                # Publish the new target
                joints, _ = self._process_angles(
                    self._trainer.TRAINING_TARGETS[index+1])
                self._publish_target(joints)
                rospy.loginfo(f"renewing recording in {time_left}")

                # Begin collecting data
                if time_left > 5:
                    self._trainer.is_calibrating = True

            elif self._trainer.is_complete:
                # If sufficient data for all gestures have been collected

                # Save the theta values stored in model
                self._model.save_theta()

                # End the while loop
                self._rate.sleep()
                break

            self._rate.sleep()

def main() -> None:
    """The main application process."""

    # Initialise ROS node
    rospy.init_node('stretchsenseCAP', anonymous=True)

    # Instantiate a ROSHandler object
    ros_handler = ROSHandler()

    if ros_handler.connect():
        # If a peripheral has been connected to
        if ros_handler.requires_calibration():
            # If calibration is required, begin calibration
            ros_handler.calibrate()
        
        # Publish the peripheral's inputs
        ros_handler.publish_input()
    

if __name__ == "__main__":
    main()