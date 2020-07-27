#!/usr/bin/env python
import time
import numpy as np
import signal
# ROS
import rospy, tf
from collections import deque
from sensor_msgs.msg import JointState
from stretchsense.msg import ssCap
from lib import StretchSense, TrainingData, SolveLeastSquares
from bluepy import btle
import binascii
import csv
import os.path

toRad = 0.01745329251

haveTheta = False
arr = 0.0

Solver = SolveLeastSquares.SolveLeastSquares()
Training = TrainingData.TrainingData()
# prefPeri = 'de:9c:60:51:f5:bf'
handle = 16
prefPeri = 'F8:A6:1C:CB:ED:A6'
# change sampling rate to 90Hz
srate = b'\x5a' #
notif = b'\x01\x00'
thetafile = "/home/husky/borealis_ws/src/stretchsense/src/theta_ss_zy.csv"
val = ''
timeout = 5
delay = 1

aD = deque(maxlen = 5)
class StretchSenseDelegate(btle.DefaultDelegate):
    def __init__(self, params):
        btle.DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        global val
        decimalValue = (binascii.b2a_hex(data))
        splitted = [decimalValue[i:i+4] for i in range(0, len(decimalValue),4)]
        val = np.array(list(map(lambda x: int((x),16)/10, splitted)))


def glove_not_found(signum, frame):
    print('Please TURN ON the glove and try again')
    quit()



def mainBLE():
    global haveTheta, val
    pub = rospy.Publisher('stretchSenseCap_10Channel', ssCap, queue_size=2)
    pubjs = rospy.Publisher('/joint_states', JointState, queue_size=2)
    # publish capacitance array
    rospy.init_node('stretchsenseCAP', anonymous=True)

    rate = rospy.Rate(100)
    Position = ssCap()
    j = JointState()

    trainingY = Training.getTrainingData()

    br = tf.TransformBroadcaster()

    def digits(position):

        # index = position[4] * toRad
        # middle = position[5] * toRad
        # ring = position[6] * toRad
        # pinky = position[7] * toRad
        # thumb = position[3] * toRad
        # splay1 = position[2] * toRad
        # splay2 = position[1] * toRad

        index = position[2] * toRad
        middle = position[3] * toRad
        ring = position[4] * toRad
        pinky = position[5] * toRad
        thumb = position[1] * toRad
        splay2 = position[0] * toRad

        digits = [splay2, thumb, thumb*0, index, index, index, middle, middle, middle, ring, ring, ring, pinky,
                  pinky, pinky]

        aD.append(digits)
        if len(aD) == 5:
            digits = [(a + b + c + d + e) / 3 for a, b, c, d, e in zip(aD[0], aD[1], aD[2], aD[3], aD[4])]
        # print(digits)
        for i in range(0, len(digits)):
            if i in [0, 1]:
                # digits[i] = digits[i] * -1.0
                continue
            if digits[i] > 0:
                digits[i] = 0
            elif digits[i] < -1.57:
                digits[i] = -1.57

        j.position = digits
        pubjs.publish(j)

        Position.values = [thumb, index, middle, ring, pinky]
        pub.publish(Position)

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
                    # theta.append([float(row[0]), float(row[1])])
                    theta.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
            theta = np.array(theta)
            TrainingData.mtheta = theta

    while not rospy.is_shutdown():
        try:
            if p.waitForNotifications(1.0):
                # MoCap Glove Skin
                # decimalValue = (binascii.b2a_hex(value))
                # splitted = [decimalValue[i:i + 4] for i in range(0, len(decimalValue), 4)]
                # cap = np.array(list(map(lambda x: int((x), 16) / 10, splitted)))
                cap = val
                idx = np.nonzero(cap)
                cap = cap[idx]#[:-1]
                cap = cap[:10].tolist()

                global old

                j.header.seq += 1
                j.header.stamp = rospy.Time.now()
                j.header.frame_id = "ssFingers"
                j.name = ["metacarpal_thumb_splay_2", "thumb_meta_prox",
                          "thumb_prox_inter", "metacarpal_index", "index_prox_inter", "index_inter_dist",
                          "metacarpal_middle", "middle_prox_inter", "middle_inter_dist", "metacarpal_ring",
                          "ring_prox_inter", "ring_inter_dist", "metacarpal_pinky", "pinky_prox_inter", "pinky_inter_dist"]
                Position.header.seq += 1
                Position.header.stamp = rospy.Time.now()
                Position.header.frame_id = 'stretch_sense_'
                Position.name = 'Capacitance'

                if TrainingData.CaptureCalibrationData == True:
                    old = time.time()
                    index = TrainingData.TrainingIndex
                    digits(trainingY[index])
                    Training.Update(cap)
                    rospy.loginfo('reading %f' % index)
                elif TrainingData.CaptureCalibrationData == False and TrainingData.complete == False:
                    timeLeft = time.time() - old
                    digits(trainingY[index + 1])
                    rospy.loginfo('renewing recording in %f' % timeLeft)
                    if timeLeft > 5:
                        TrainingData.CaptureCalibrationData = True

                elif TrainingData.complete == True:
                    if haveTheta == False:
                        with open(thetafile, 'w') as csvfile:
                            thetawriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                            thetawriter.writerows(TrainingData.mtheta)
                        haveTheta = True

                    # rospy.loginfo('Publishing data in topic /stretchSenseCap_10Channel')
                    cap = np.insert(cap, 0, 1)
                    Position.values = Solver.ApplyTransformation(cap, TrainingData.mtheta)
                    digits(Position.values)
            else:
                print('waiting...')
        except:
            print('disconnecting...')
            p.disconnect()
            quit()
            pass


if __name__ == "__main__":
    signal.signal(signal.SIGALRM, glove_not_found)
    signal.alarm(3)

    try:
        p = btle.Peripheral(prefPeri, 'random')
        p.setDelegate(StretchSenseDelegate(p))
        svc = p.getServiceByUUID('00001701-7374-7265-7563-6873656e7365')
        char = svc.getCharacteristics()[0]
        handle = char.valHandle
        p.writeCharacteristic(handle + 1, b'\x01\x00')  # turn on notifications
        p.writeCharacteristic(29, srate)  # change sampling rate to 90Hz

    except rospy.ROSInterruptException:
        p.disconnect()
        pass

    signal.alarm(0)
    mainBLE()

