#!/usr/bin/env python3
import numpy as np
from lib import SolveLeastSquares
import time

Solver = SolveLeastSquares.SolveLeastSquares()
# gloves 7 sensors left hand
# Thumb,Thumb,Thumb,Thumb,Index,Middle,Ring,Pinky
# TrainingY = [
#   [ 0, -15, 0, 40, 0, -80, -80, -80], #Point index
#   [ 0, -15, 0, 40, -80, -50, -80, -80], #Point middle
#   [ 0, 0, 0, -15, -90, -90, -90, -90], #Close fist
#   [ 0, -15, -60, 0, -90, -90, -90, -90], #Thumb up
#   [ 0, -15, -60, 0, -90, -90, -90, 0], #Thumb up, pinky out
#   [ 0, -15, -60, 0, 0, 0, 0, 0], #Open hand
#   [ 0, -15, 0, 40, 0, 0, 0, 0], #Tuck thumb
#   [ 0, -15, 0, 40, -80, -80, -80, -80], #Tuck thumb in fist
#   [ -35, 55, -65, 30, -55, 0, 0, 0], #ok sign
#   [ -60, 33, -35, 30, 0, -60, 0, 0], #touch middle finger
#   [ -60, 33, -35, 30, 0, -60, -60, 0], #touch middle finger and ring finger
#   [ 0, -15, -60, 0, 0, 0, 0, 0], #Spread hand open (target thumb)
#   [ 0, 30, -60, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   [ 0, 60, -60, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   [ 0, 60, -45, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   [ 0, 15, -15, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   [ 0, 0, 0, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   [ -15, 90, -45, 0, -90, -90, -90, -90], #Thumb over fist
# ]


# gloves 7 sensors left hand
# Thumb,Thumb,Thumb,Thumb,Index,Middle,Ring,Pinky
# TrainingName = ["close fist", "thumb up", "Thumb up, pinky out", "Open hand", "tuck thumb", "tuck thumb in fist", "ok sign", "touch middle finger",
#                 "touch middle finger and ring finger", "Spread hand open (target thumb)", "Rotate thumb hand open (target thumb)","Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)",
#                 "Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)", "Thumb over fist"]

# TrainingName = ["close fist", "thumb up", "Thumb up, pinky out", "Open hand", "tuck thumb", "tuck thumb in fist", "ok sign", "touch middle finger",
#                 "touch middle finger and ring finger", "Spread hand open (target thumb)", "Rotate thumb hand open (target thumb)","Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)",
#                 "Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)", "Thumb over fist"]

# TrainingY = [
#   [ 0, 0, 0, -5, -90, -90, -90, -90], #Close fist
#   [ 0, -15, -70, 0, -90, -90, -90, -90], #Thumb up
#   [ 0, -15, -70, 0, -90, -90, -90, 0], #Thumb up, pinky out
#   [0, -15, -70, 0, 0, -90, -90, -90],  # Thumb up, index out
#   [0, -15, -70, 0, -90, 0, -90, -90],  # Thumb up, middle out
#   [0, -15, -70, 0, -90, -90, -45, -90],  # Thumb up, ring out
#   [ 0, -15, -70, 0, 0, 0, 0, 0], #Open hand
#   [ 0, -15, 0, 40, 0, 0, 0, 0], #Tuck thumb
#   [ 0, -15, 0, 40, -80, -80, -80, -80], #Tuck thumb in fist
#   # [ -35, 55, -65, 30, -55, 0, 0, 0], #ok sign
#   # [ -70, 33, -35, 30, 0, -70, 0, 0], #touch middle finger
#   # [ -70, 33, -35, 30, 0, -70, -70, 0], #touch middle finger and ring finger
#   # [ 0, -15, -70, 0, 0, 0, 0, 0], #Spread hand open (target thumb)
#   # [ 0, 30, -70, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   # [ 0, 60, -70, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   # [ 0, 60, -45, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   # [ 0, 15, -15, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   # [ 0, 0, 0, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
#   # [ -15, 90, -45, 0, -90, -90, -90, -90], #Thumb over fist
# ]

TrainingY = [
  [ 0, -5, -90, -90, -90, -90], #Close fist
  [ 0, -45, -90, -90, -90, -90], #Thumb up 45 deg
  # [ 0, -80, -90, -90, -90, -90], #Thumb up 85 deg
  [ 0, -45, -90, -90, -90, 0], #Thumb up, pinky out
  [0, -45, 0, -90, -90, -90],  # Thumb up, index out
  [0, -45, -90, 0, -90, -90],  # Thumb up, middle out
  # [0, -45, -90, -90, -45, -90],  # Thumb up, ring out
  [ 0, -45, 0, 0, 0, 0], #Open hand
  [ 0, 30, 0, 0, 0, 0], #Tuck thumb
  # [ 0, 30, -80, -80, -80, -80], #Tuck thumb in fist
  [ -45, 0, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
  [ -45, 30, 0, 0, 0, 0], #Rotate thumb hand open (target thumb)
  [ 0, 0, 0, 0, 0, 0], #Thumb 0 0
  [ -45, 30, -90, -90, -90, -90] #Thumb over fist
]



#left hand
# TrainingY = [
#    [0, -80, -80, -80, 0, 40, 0, -15], #Point index
#    [0, -80, -80, 0, -80, 40, 0, -15], #Point middle
#    [0, -80, -80, 0, 0, 40, 0, -15],
#    [0, -80, -80, -80, -80, 40, 0, -15]
# ];

All = [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ]
ActiveSensor =  [All, All, All, All, All, All, All, All]

#booleans
CaptureCalibrationData = True
FirstPress = True
complete = False
storageKey = 'Calibration'

#indices
TrainingIndex = 0
samplesIndex = 0
index = 0

#array for solver
nSensors = 7 #smart glove
nOutputs = len(TrainingY[0])
spp = 200
size = spp*len(TrainingY)
X = np.zeros((size,nSensors+1))
y = np.zeros((size,nOutputs))
LastX = [0]*5
mtheta = []

currentY = []

samplesPerPosition = spp
TrainingPositions = len(TrainingY)
#TrainingPositions = len(TrainingY)

nSamples = samplesPerPosition * TrainingPositions;





class TrainingData:
    
    def getTrainingMax(self, TrainingY):
        TrainY = []
        for i in range(0, len(TrainingY[0])):
                       TrainY.append(TrainingY[0][i])
                       for j in range(0,len(TrainingY)):
                            if TrainY[i] < TrainingY[j][i]:
                                   TrainY[i] = TrainingY[j][i]

        return TrainY
                       
                       
    def getTrainingMin(self, TrainingY):
        TrainY = []
        for i in range(0, len(TrainingY[0])):
                       TrainY.append(TrainingY[0][i])
                       for j in range(0,len(TrainingY)):
                            if TrainY[i] > TrainingY[j][i]:
                                   TrainY[i] = TrainingY[j][i]

        return TrainY

                       
    def getCurrentY(self, TrainingY):
        current = []
        for i in range(0, len(TrainingY[0])):
                       current.append(TrainingY[0][i])
                                      
        return current
                                      
    def RecordNow(self):
        #Require 2 presses to start first capture (TODO determine more stable way to manage first press)
        if index == 0 and FirstPress:
            FirstPress = False;
        else:
            CaptureCalibrationData = True;
        
    def getTrainingData(self):
        return TrainingY

    def Update(self, currentX):
        global LastX, CaptureCalibrationData, X, y, complete, samplesIndex, index, currentY
        if len(currentY) == 0:
            currentY = self.getCurrentY(TrainingY)
        if complete:
            return
        if currentX == LastX:
            return
        else:
            LastX = currentX
            if CaptureCalibrationData:
                X[index][0] = 1
                for i in range(0, nSensors):
                      X[index][i + 1] = currentX[i]

                for j in range(0, nOutputs):
                      y[index][j] = currentY[j]

                index += 1
                samplesIndex += 1
                if samplesIndex >= samplesPerPosition:
                    # print(X[0])
                    # print('\n')
                    # print(Y[0])
                    #Update to new position
                    self.UpdatePosition()
                    #Reset Flags
                    CaptureCalibrationData = False
                    #Reset counters
                    samplesIndex = 0
                                      
                                      
        
    def UpdatePosition(self):
        global complete, TrainingIndex, TrainingPositions, nOutputs, currentY
        if complete:
            return

        TrainingIndex += 1
        
        if TrainingIndex < TrainingPositions:
            for i in range(0, nOutputs):
                currentY[i] = TrainingY[TrainingIndex][i]

            print('next position in 5 seconds')

        else:
            print('Done recording')
            complete = True
            storageKey = 'Calibration'
            #Solver.ActiveSensors(ActiveSensor)
            trainingData = Solver.Solve(X,y)
            self.setTheta(trainingData)
        
    def ApplyTransformation(self, inputt, theta):
        return Solver.ApplyTransformation(inputt, theta)
    def setTheta(self, theta):
        global mtheta
        mtheta = theta
    def getTheta(self):
        return mtheta
    def getCurrentPosition(self):
        return currentY
    def isComplete(self):
        return complete
    def setComplete(self):
        complete = isComplete()
    def getX(self):
        return X
    def getY(self):
        return y                          


##Training = TrainingData()
##TrainingYMax = Training.getTrainingMax(TrainingY)
##TrainingYMin = Training.getTrainingMin(TrainingY);
##currentY = Training.getCurrentY(TrainingY);
##
##print(TrainingYMax, TrainingYMin, currentY)

    

    

    


