#!/usr/bin/env python3
import time
import numpy as np

#ROS
import rospy
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg import Header
from stretchsense.msg import ssCap
import sys
from lib import StretchSense, TrainingData, SolveLeastSquares

stretchsenseObject = StretchSense.StretchSenseAPI()
Solver = SolveLeastSquares.SolveLeastSquares()
Training = TrainingData.TrainingData()
prefPeri = 'de:9c:60:51:f5:bf'


def stretchsenseBLE():
    pub = rospy.Publisher('stretchSenseCap_10Channel', ssCap, queue_size=10)
    rospy.init_node('stretchsenseCAP', anonymous=True)
    rate = rospy.Rate(100)
    def updateValue():
        global old
        stretchsenseObject.ble_waitNotifications()
        if TrainingData.CaptureCalibrationData == True:
            Training.Update(stretchsenseObject.ble_getValuesCsv())
            old = time.time()
            rospy.loginfo('recording data')
        elif TrainingData.CaptureCalibrationData == False and TrainingData.complete == False:
            timeLeft = time.time() - old
            rospy.loginfo('renewing recording in %f' % timeLeft)
            if  timeLeft > 5:
                TrainingData.CaptureCalibrationData = True
        elif TrainingData.complete == True:
            rospy.loginfo('Publishing data in topic /stretchSenseCap_10Channel')
            inputt = np.array([1])
            inputt = np.append(inputt, stretchsenseObject.ble_getValuesCsv())

            Position.header.seq += 1
            Position.header.stamp = rospy.Time.now()
            Position.header.frame_id = 'stretch_sense_'
            Position.name = 'Capacitance'
            Position.values = Solver.ApplyTransformation(inputt[0:8], TrainingData.mtheta)

            pub.publish(Position)

    #find if preferred Stretch Sense Device is available
    print('scanning')
    stretchsenseObject.ble_scanning(3)                                          #scan for 3 seconds
    availablePeripherals = stretchsenseObject.ble_getListPeripheralAvailable()  #list of available peripherals

    for i in availablePeripherals:
        if i.addr == prefPeri:
            rospy.loginfo("Preferred Peripheral Available")
            stretchsenseObject.ble_connectOnePeripheral(prefPeri)
            numberOfPeripheralConnected = len(stretchsenseObject.ble_getListPeripheralIsConnected())
            if stretchsenseObject.ble_getListPeripheralIsConnected()[0].addr == prefPeri:
                Position = ssCap()
                while not rospy.is_shutdown():
                    try:
                        updateValue()
                        rate.sleep()
                    except:
                        pass
            else:
                rospy.loginfo("Connection Failed")
        else:
            rospy.loginfo("Preferred Peripheral not found")
            

##    if numberOfPeripheralConnected > 0:
##            rospy.loginfo('found preferred peripheral')
##            #t = StretchSense.RepeatedTimer(0.01, lambda: updateValue())
##            pub = rospy.Publisher('/stretchSenseCap_10Channel', ssCap, queue_size=10)
##            rospy.init_node('stretchsense', anonymous=False)
##            rate = rospy.Rate(100)
##            Position = ssCap()
##            while not rospy.is_shutdown():
##                try:
##                    updateValue()
##                    rate.sleep()
##                except:
##                    pass
##    else:
##        rospy.loginfo('no preferred')
##        pass
    

            
            

    
if __name__ == "__main__":
    try:
        stretchsenseBLE()
    except rospy.ROSInterruptException:
        pass
