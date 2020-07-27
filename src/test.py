#!/usr/bin/env python
import time
import numpy as np

#ROS
import rospy, tf
from sensor_msgs.msg import ChannelFloat32, JointState
from std_msgs.msg import Header, Float64MultiArray
from stretchsense.msg import ssCap
import sys
from lib import StretchSense, TrainingData, SolveLeastSquares
import bluepy
import binascii
import csv
import os.path
from joblib import load

toRad = 0.01745329251

haveTheta = False
arr = 0.0
stretchsenseObject = StretchSense.StretchSenseAPI()
Solver = SolveLeastSquares.SolveLeastSquares()
Training = TrainingData.TrainingData()
# prefPeri = 'de:9c:60:51:f5:bf'
handle = 16
prefPeri = 'F8:A6:1C:CB:ED:A6'
# change sampling rate to 90Hz
srate = b'\x5a'
ss = bluepy.btle.Peripheral()

ss.connect(prefPeri, 'random')
ss.writeCharacteristic(29, srate)


thetafile = "/home/husky/borealis_ws/src/stretchsense/src/theta_ss.csv"
def mainBLE():
    global haveTheta
    pub = rospy.Publisher('stretchSenseCap_10Channel', ssCap, queue_size=10)
    pubjs = rospy.Publisher('/joint_states', JointState, queue_size=10)
    #publish capacitance array
    rospy.init_node('stretchsenseCAP', anonymous=True)


    rate = rospy.Rate(90)
    Position = ssCap()
    j = JointState()
    trainingY = Training.getTrainingData()

    br = tf.TransformBroadcaster()

    def callback(data):
        global arr
        arr = data.data[0]

    def digits(position):

        index = position[4] * toRad
        middle = position[5] * toRad
        ring = position[6] * toRad
        pinky = position[7] * toRad

        thumb = position[3] * toRad
        splay1 = position[2] * toRad
        splay2 = position[1] * toRad


        digits = [splay1, splay2, thumb, thumb, index, index, index,  middle, middle, middle, ring, ring, ring, pinky, pinky, pinky]
        # print(digits)
        for i in range(0, len(digits)):
            if i in [2,3]:
                digits[i] = digits[i] * -1.0
                continue
            if digits[i] > 0:
                digits[i] = 0
            elif digits[i] < -1.57:
                digits[i] = -1.57


        j.position = digits
        pubjs.publish(j)

        Position.values = position
        pub.publish(Position)


    print(os.path.isfile(thetafile))
    if os.path.isfile(thetafile):
        cal = raw_input('Calibrate Again? Y/N ')
        if cal == 'Y' or cal == 'y':
            haveTheta = False
            pass
        else:
            theta = []
            haveTheta = True
            TrainingData.CaptureCalibrationData = False
            TrainingData.complete = True
            with open(thetafile) as csvfile:
                thetareader = csv.reader(csvfile, delimiter=',', quotechar='|')
                for row in thetareader:
                    theta.append([float(row[0]), float(row[1])])
            theta = np.array(theta)
            TrainingData.mtheta = theta

    while not rospy.is_shutdown():
        try:

            # cap = []
            value = ss.readCharacteristic(handle)

            # MoCap Glove Skin
            decimalValue = (binascii.b2a_hex(value))
            splitted = [decimalValue[i:i + 4] for i in range(0, len(decimalValue), 4)]
            cap = np.array(list(map(lambda x: int((x),16)/10, splitted)))
            idx = np.nonzero(cap)
            cap = cap[idx][:-1]
            cap = cap[:10].tolist()
            # print(cap)

            # Smart Glove
            # for i in range(0,19,2):
            #     decimalValue = int(binascii.b2a_hex(value[i:i+2]),16)/10
            #     cap.append(decimalValue)
            #
            # print(cap)

            
            global old

            j.header.seq += 1
            j.header.stamp = rospy.Time.now()
            j.header.frame_id = "ssFingers"
            j.name = ["metacarpal_thumb_splay_1", "metacarpal_thumb_splay_2", "thumb_meta_prox",
                      "thumb_prox_inter", "metacarpal_index", "index_prox_inter", "index_inter_dist",
                      "metacarpal_middle", "middle_prox_inter", "middle_inter_dist", "metacarpal_ring",
                      "ring_prox_inter", "ring_inter_dist", "metacarpal_pinky", "pinky_prox_inter", "pinky_inter_dist"]
            Position.header.seq += 1
            Position.header.stamp = rospy.Time.now()
            Position.header.frame_id = 'stretch_sense_'
            Position.name = 'Capacitance'

            if TrainingData.CaptureCalibrationData == True:
                Training.Update(cap)
                old = time.time()
                index = TrainingData.TrainingIndex
                digits(trainingY[index])
                # rospy.loginfo(trainingY[index])

            elif TrainingData.CaptureCalibrationData == False and TrainingData.complete == False:
                timeLeft = time.time() - old
                index = TrainingData.TrainingIndex
                rospy.loginfo('renewing recording in %f' % timeLeft)
                # rospy.loginfo('renewing recording for %s in %f' % (TrainingData.TrainingName[index], timeLeft))
                if timeLeft > 2:
                    TrainingData.CaptureCalibrationData = True

            elif TrainingData.complete == True:
                if haveTheta == False:
                    with open(thetafile, 'w') as csvfile:
                        thetawriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                        thetawriter.writerows(TrainingData.mtheta)
                    haveTheta = True

                rospy.Subscriber('vnavMagnitude', Float64MultiArray, callback)
                #rospy.loginfo('Publishing data in topic /stretchSenseCap_10Channel')
                cap = np.insert(cap, 0, 1)
                Position.values = Solver.ApplyTransformation(cap, TrainingData.mtheta)
                digits(Position.values)

            rate.sleep()
        except:
            # rospy.loginfo('error')
            pass

        
    
if __name__ == "__main__":
    try:
        mainBLE()
    except rospy.ROSInterruptException:
        pass
