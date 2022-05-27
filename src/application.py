"""The main application script."""

import time
from typing import List, Optional, Tuple
import numpy as np
import math

import pandas as pd

import rospy, rospkg
from sensor_msgs.msg import JointState
from stretchsense.msg import ssCap

from peripheral import bluetooth_handler, stretchsense_peripheral
from model import model, trainer

rospack = rospkg.RosPack()

class ROSHandler:
    """This class handles publishing data to ROS."""

    _PACKAGE_DIRECTORY = rospack.get_path('stretchsense')
    _TO_RAD = math.pi / 180

    def __init__(self):
        # For connecting to peripheral
        self._bluetooth_handler = bluetooth_handler.BluetoothHandler(
            ROSHandler._PACKAGE_DIRECTORY)
        self._glove: stretchsense_peripheral.StretchSensePeripheral

        # For machine learning
        self._model: model.Model
        self._trainer: trainer.Trainer

        # For publishing to ROS
        self._joint_publisher = rospy.Publisher('/Joints',
                                                JointState,
                                                queue_size=2)
        self._finger_publisher = rospy.Publisher('/Fingers',
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

    def connect(self) -> None:
        """Connect to peripheral and set up publisher."""

        glove = self._bluetooth_handler.connect_glove()
        if glove:
            self._joint_states.name = [glove.get_side()
                                + "_"
                                + name for name in self._joint_states.name]
            self._glove = glove
            self._model = model.Model(self._glove)
            self._trainer = trainer.Trainer(self._model)

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
        
    def _publish_target(self, target_angles: np.ndarray) -> None:
        """Publishes target angles during calibration for user to follow.
        
        Takes in a numpy array of target angles, processes them, then publishes
        the joint data.
        
        Args:
            target_angles:
                numpy array representing each finger's angles in degrees
        """

        joints, _ = self._process_angles(target_angles)
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
            sensor_data = self._glove.read_sensors()
            if not sensor_data:
                return
            sensor_data = np.insert(sensor_data, 0, 1)
            transformed = self._model.apply_transformation(sensor_data)
            joints, fingers = self._process_angles(transformed)
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
            sensor_data = self._glove.read_sensors()
            if not sensor_data:
                return
            
            if self._trainer.is_calibrating:
                    old = time.time()
                    index = self._trainer.gesture_index
                    joints, _ = self.process_angles(
                        self._trainer.TRAINING_TARGETS[index])
                    self._publish_target(joints)
                    self._trainer.update_sample(sensor_data)
                    rospy.loginfo(f"reading: {index}")

            elif (not self._trainer.is_calibrating and
                  not self._trainer.is_complete):
                time_left = time.time() - old
                joints, _ = self._process_angles(
                    self._trainer.TRAINING_TARGETS[index+1])
                self._publish_target(joints)
                rospy.loginfo(f"renewing recording in {time_left}")
                if time_left > 5:
                    self._trainer.is_calibrating = True

            elif self._trainer.is_complete:
                filepath = (self.PACKAGE_DIRECTORY
                             + "/src/data/theta_"
                             + str(rospy.Time.now())
                             + ".csv")
                self._model.save_theta(filepath)

            self._rate.sleep()

def main() -> None:
    """The main application process."""

    rospy.init_node('stretchsenseCAP', anonymous=True)
    ros_handler = ROSHandler()

    if ros_handler.connect():
        if ros_handler.requires_calibration():
            ros_handler.calibrate()
        ros_handler.publish_input()
    

if __name__ == "__main__":
    main()