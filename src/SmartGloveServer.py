#! /usr/bin/env python3

import roslib

roslib.load_manifest('stretchsense')
from threading import Thread
from multiprocessing.pool import ThreadPool
import rospy
import actionlib
import time
from stretchsense.msg import smartGloveServerAction, smartGloveServerResult, smartGloveServerGoal, \
    smartGloveServerFeedback
from SmartGloveHandler import SmartGloveHandler


class SmartGloveServer:
    def __init__(self):
        self.myglove = SmartGloveHandler()
        self.sensor_read_thread = None
        self.stop_child_thread = False

        self.feedback = smartGloveServerFeedback()
        self.result = smartGloveServerResult()
        self.res = []
        self.server = actionlib.SimpleActionServer('smartglove', smartGloveServerAction, self.execute, False)
        self.server.start()

    def stop_thread(self):
        if isinstance(self.sensor_read_thread, ThreadPool):
            print('calling stop')
            self.myglove.peripheralInUse.delegate.stop_thread = True
            self.sensor_read_thread.terminate()
            self.sensor_read_thread.join()

    def start_publisher_thread(self, function: callable):
        # stop any prior thread
        self.stop_thread()
        self.sensor_read_thread = ThreadPool(processes=1)
        self.sensor_read_thread.apply_async(function)

    def scan(self) -> list:
        # return the list of devices found
        self.feedback.feedback = 'scanning...'
        if self.myglove.findGloves():
            kP = self.myglove.knownPeripherals
            aP = self.myglove.availablePeripherals
            known_peripherals = [value for value in kP.values()]
            return known_peripherals + aP
        else:
            return ['No compatible gloves found']

    def connect(self, addr: str) -> bool:
        return self.myglove.actionServerConnectGlove(addr)

    def execute(self, goal: smartGloveServerGoal) -> None:
        received = goal.goal
        thegoal = received[0]
        carry = received[-1]

        if thegoal == 'scan':
            self.res = self.scan()

        elif thegoal == 'connect':
            self.stop_thread()
            self.myglove.disconnectPeripheral()

            if self.connect(carry):
                data_files = self.myglove.findTrainedData()
                self.res = ['Connection Successful!'] + data_files
            else:
                self.res = ['Connection Failed']

        elif thegoal == 'select_theta':
            if self.myglove.actionServerSelectData(carry):
                self.start_publisher_thread(self.myglove.callPublisher)
                self.res = ['Model Selected']
            else:
                self.res = ['Error selecting file']

        elif thegoal == 'calibrate':
            self.start_publisher_thread(self.myglove.calibrateGlove)
            self.res = ['Clench your fist and follow the model on the screen']

        elif thegoal == 'stop':
            print('called stop')
            self.stop_thread()
            self.res = ['thread stopped']

        self.result.result = self.res
        self.server.set_succeeded(self.result)


if __name__ == '__main__':
    try:
        rospy.init_node('stretchsenseCAP')
        server = SmartGloveServer()
        rospy.spin()
    except KeyboardInterrupt as e:
        print(e)
        exit()
