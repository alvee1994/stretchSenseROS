#!/usr/bin/env python3
import numpy as np
from . import SolveLeastSquares

Solver = SolveLeastSquares.SolveLeastSquares()

# gloves 7 sensors left hand
# Thumb,Thumb,Thumb,Thumb,Index,Middle,Ring,Pinky
# TrainingName = ["close fist", "thumb up", "Thumb up, pinky out", "Open hand", "tuck thumb", "tuck thumb in fist", "ok sign", "touch middle finger",
#                 "touch middle finger and ring finger", "Spread hand open (target thumb)", "Rotate thumb hand open (target thumb)","Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)",
#                 "Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)", "Thumb over fist"]

# TrainingName = ["close fist", "thumb up", "Thumb up, pinky out", "Open hand", "tuck thumb", "tuck thumb in fist", "ok sign", "touch middle finger",
#                 "touch middle finger and ring finger", "Spread hand open (target thumb)", "Rotate thumb hand open (target thumb)","Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)",
#                 "Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)", "Thumb over fist"]

class TrainingData:

    TrainingY = np.array([
        [0, -5, -90, -90, -90, -90],  # Close fist
        [0, -45, -90, -90, -90, -90],  # Thumb up 45 deg
        # [ 0, -80, -90, -90, -90, -90], #Thumb up 85 deg
        [0, -45, -90, -90, -90, 0],  # Thumb up, pinky out
        [0, -45, 0, -90, -90, -90],  # Thumb up, index out
        # [0, -45, -90, 0, -90, -90],  # Thumb up, middle out
        # [0, -45, -90, -90, -45, -90],  # Thumb up, ring out
        [0, -45, 0, 0, 0, 0],  # Open hand
        [0, 30, 0, 0, 0, 0],  # Tuck thumb
        # [ 0, 30, -80, -80, -80, -80], #Tuck thumb in fist
        [-45, 0, 0, 0, 0, 0],  # Rotate thumb hand open (target thumb)
        [-45, 30, 0, 0, 0, 0],  # Rotate thumb hand open (target thumb)
        [0, 0, 0, 0, 0, 0],  # Thumb 0 0
        [-45, 30, -90, -90, -90, -90]  # Thumb over fist
    ])

    def __init__(self):
        # booleans
        self.CaptureCalibrationData = True
        self.FirstPress = True
        self.complete = False

        # indices
        self.TrainingIndex = 0
        self.gestIndex = 0
        self.smplidx = 0

        # array for solver
        self.nSensors = 7  # smart glove
        self.nGestures, self.nOutputs = self.TrainingY.shape
        self.samplesPerPosition = 100
        self.size = self.samplesPerPosition * self.nGestures #200 samples per gesture
        self.X = np.zeros((self.size, self.nSensors + 1))
        self.y = np.zeros((self.size, self.nOutputs))

        self.LastX = np.zeros(self.nSensors)
        self.mtheta = []
        self.currentY = self.TrainingY[0]

        self.TrainingPositions = self.nGestures
        self.nSamples = self.size

    def getTrainingData(self):
        return self.TrainingY

    def Update(self, currentX):

        compare = np.array([currentX == self.LastX])

        if self.complete:
            return

        print(compare, type(currentX), type(self.LastX), type(compare))
        if compare.all():
            print('no new data received\n')
            return
        else:
            self.LastX = currentX
            if self.CaptureCalibrationData:
                self.X[self.smplidx][0] = 1
                self.X[self.smplidx][1:] = np.array(currentX)
                self.y[self.smplidx] = self.currentY

                self.smplidx += 1
                self.gestIndex += 1
                if self.gestIndex >= self.samplesPerPosition:
                    #Update to new position
                    self.UpdatePosition()
                    #Reset Flags
                    self.CaptureCalibrationData = False
                    #Reset counters
                    self.gestIndex = 0
                                      
                                      
        
    def UpdatePosition(self):
        if self.complete:
            return

        self.TrainingIndex += 1
        
        if self.TrainingIndex < self.TrainingPositions:
            self.currentY = self.TrainingY[self.TrainingIndex]
            print('next position in 5 seconds')

        else:
            print('Done recording')
            self.complete = True
            trainingData = Solver.Solve(self.X,self.y)
            self.setTheta(trainingData)
        
    def ApplyTransformation(self, inputt, theta):
        return Solver.ApplyTransformation(inputt, theta)
    def setTheta(self, theta):
        self.mtheta = theta


    

    


