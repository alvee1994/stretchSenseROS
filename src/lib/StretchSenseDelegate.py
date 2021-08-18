#!/usr/bin/env python3

from typing import Tuple, List
import time
import rospy
import numpy as np
import pandas as pd
import binascii
from bluepy import btle
from sensor_msgs.msg import JointState
from lib import TrainingData, SolveLeastSquares


# from stretchsense.msg import ssCap


class StretchSenseDelegate(btle.DefaultDelegate):
    def __init__(self, params):
        btle.DefaultDelegate.__init__(self)

        self.capacitance = []
        self.toRad = 0.01745329251

        # ROS
        self.rate = rospy.Rate(100)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=2)
        self.Joints = JointState()
        self.Joints.header.frame_id = "ssFingers"
        self.Joints.name = ["left_" + string for string in ["metacarpal_thumb_splay_2", "thumb_meta_prox",
                                                            "thumb_prox_inter", "metacarpal_index", "index_prox_inter",
                                                            "index_inter_dist", "metacarpal_middle",
                                                            "middle_prox_inter",
                                                            "middle_inter_dist", "metacarpal_ring",
                                                            "ring_prox_inter", "ring_inter_dist", "metacarpal_pinky",
                                                            "pinky_prox_inter", "pinky_inter_dist"]]

        # for a gesture recognizer to subscribe to
        # self.pubfingers = rospy.Publisher('/fingers', ssCap, queue_size=2)
        # self.Position = ssCap()

        # linear algebra
        self.Solver = SolveLeastSquares.SolveLeastSquares()
        self.Training = TrainingData.TrainingData()
        self.trainingY = self.Training.getTrainingData()
        self.mtheta = None

        # calibrateSensors()
        self.old = -1
        self.trainingIndex = -1
        self.timeLeft = -1
        self.trainingDone = False

    def handleNotification(self, cHandle, data):
        decimalValue = (binascii.b2a_hex(data))
        splitted = [decimalValue[i:i + 4] for i in range(0, len(decimalValue), 4)]
        val = np.array(list(map(lambda x: int((x), 16) / 10, splitted)))
        idx = np.nonzero(val)
        cap = val[idx]
        self.capacitance = np.insert(cap, 0, 1)

    def applyTransformation(self) -> np.ndarray:
        return self.Solver.ApplyTransformation(self.capacitance, self.mtheta)

    def degToRads(self, position: np.ndarray):
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

    def setTheta(self, theta):
        self.mtheta = theta
        self.Training.CaptureCalibrationData = False
        self.Training.complete = True
        self.Training.setTheta(theta)

    def calibrateSensors(self, callback):
        while not self.trainingDone and not rospy.is_shutdown():
            callback()
            if self.Training.CaptureCalibrationData:
                self.old = time.time()
                self.trainingIndex = self.Training.TrainingIndex
                d, _ = self.degToRads(self.trainingY[self.trainingIndex])
                self.publishCapacitance(calibrate=True, digits=d)
                self.Training.Update(self.capacitance)
                # rospy.loginfo(f'reading {self.trainingIndex}')
            elif self.Training.CaptureCalibrationData is False and self.Training.complete is False:
                self.timeLeft = time.time() - self.old
                d, _ = self.degToRads(self.trainingY[self.trainingIndex + 1])  # ignore fingers when calibrating
                self.publishCapacitance(calibrate=True, digits=d)
                rospy.loginfo(f'renewing recording in {self.timeLeft}')
                if self.timeLeft > 5:
                    self.Training.CaptureCalibrationData = True
            elif self.Training.complete:
                self.mtheta = self.Training.mtheta
                self.trainingDone = True
            self.rate.sleep()

        print("Calibration complete")
        return self.mtheta

    def publishCapacitance(self, callback=None, calibrate=False, digits=np.array([])):
        if not calibrate:
            try:
                while not rospy.is_shutdown():
                    callback()
                    sensor_values = self.applyTransformation()
                    digits, _ = self.degToRads(sensor_values)

                    self.Joints.header.seq += 1
                    self.Joints.header.stamp = rospy.Time.now()
                    self.Joints.position = digits
                    self.pub.publish(self.Joints)
                    self.rate.sleep()
            except:
                pass
        else:
            self.Joints.header.seq += 1
            self.Joints.header.stamp = rospy.Time.now()
            self.Joints.position = digits
            self.pub.publish(self.Joints)
