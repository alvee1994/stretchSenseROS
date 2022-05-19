#!/usr/bin/env python3
import numpy as np
from .SolveLeastSquares import SolveLeastSquares



# gloves 7 sensors left hand
# Thumb,Thumb,Thumb,Thumb,Index,Middle,Ring,Pinky
# TrainingName = ["close fist", "thumb up", "Thumb up, pinky out", "Open hand", "tuck thumb", "tuck thumb in fist", "ok sign", "touch middle finger",
#                 "touch middle finger and ring finger", "Spread hand open (target thumb)", "Rotate thumb hand open (target thumb)","Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)",
#                 "Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)", "Thumb over fist"]

# TrainingName = ["close fist", "thumb up", "Thumb up, pinky out", "Open hand", "tuck thumb", "tuck thumb in fist", "ok sign", "touch middle finger",
#                 "touch middle finger and ring finger", "Spread hand open (target thumb)", "Rotate thumb hand open (target thumb)","Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)",
#                 "Rotate thumb hand open (target thumb)", "Rotate thumb hand open (target thumb)", "Thumb over fist"]

class TrainingData:

    TRAINING_TARGETS = np.array([
        [0, -5, -90, -90, -90, -90],  # Close fist
        [0, -45, -90, -90, -90, -90],  # Thumb up 45 deg
        [0, -45, -90, -90, -90, 0],  # Thumb up, pinky out
        [0, -45, 0, -90, -90, -90],  # Thumb up, index out
        [0, -45, 0, 0, 0, 0],  # Open hand
        [0, 30, 0, 0, 0, 0],  # Tuck thumb
        [-45, 0, 0, 0, 0, 0],  # Rotate thumb hand open (target thumb)
        [-45, 30, 0, 0, 0, 0],  # Rotate thumb hand open (target thumb)
        [0, 0, 0, 0, 0, 0],  # Thumb 0 0
        [-45, 30, -90, -90, -90, -90]  # Thumb over fist
    ])

    def __init__(self):
        # constants
        self.NUM_SENSORS = 7
        self.NUM_GESTURES, self.NUM_OUTPUTS = self.TRAINING_TARGETS.shape
        self.SAMPLES_PER_POSITION = 100
        self.NUM_SAMPLES = self.SAMPLES_PER_POSITION * self.NUM_GESTURES

        # booleans
        self.is_calibrating = True
        self.is_complete = False

        # indices
        self.training_index = 0
        self.gesture_index = 0
        self.sample_index = 0

        self.inputs = np.zeros((self.NUM_SAMPLES, self.NUM_SENSORS + 1))
        self.targets = np.zeros((self.NUM_SAMPLES, self.NUM_OUTPUTS))

        self.prev_input = np.zeros(self.NUM_SENSORS)
        self.mtheta = []
        self.curr_target = self.TRAINING_TARGETS[0]

        self.SOLVER = SolveLeastSquares()

    def get_training_data(self):
        return self.TRAINING_TARGETS

    def update_sample(self, curr_input):
        
        if self.is_complete:
            return

        if np.array_equal(curr_input, self.prev_input):
            print('no new data received\n')
            return
        else:
            self.prev_input = curr_input
            if self.is_calibrating:
                self.inputs[self.sample_index][0] = 1
                self.inputs[self.sample_index][1:] = np.array(curr_input)
                self.targets[self.sample_index] = self.curr_target

                self.sample_index += 1
                self.gesture_index += 1
                if self.gesture_index >= self.SAMPLES_PER_POSITION:
                    # Update to new position
                    self.update_gesture()
                    # Reset Flags
                    self.is_calibrating = False
                    # Reset counters
                    self.gesture_index = 0
        
    def update_gesture(self):
        if self.is_complete:
            return

        self.training_index += 1
        
        if self.training_index < self.NUM_GESTURES:
            self.curr_target = self.TRAINING_TARGETS[self.training_index]
            print('next position in 5 seconds')

        else:
            print('Done recording')
            self.is_complete = True
            self.mtheta = self.SOLVER.solve(self.inputs, self.targets)


    

    


