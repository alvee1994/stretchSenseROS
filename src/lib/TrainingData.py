#!/usr/bin/env python3
"""Encapsulates the training data."""
import numpy as np
from .SolveLeastSquares import SolveLeastSquares

class TrainingData:
    """Handles the training of the gesture recognition models.

    Collects and stores capacitance data and targets to be passed
    to a solver to train the required models.

    Attributes:
        SOLVER:
            A regression line calculator.
        TRAINING_TARGETS: 
            A numpy array of integers where each row represents 
            one gesture and each row contains the target angles for each joint.
        NUM_SENSORS: 
            Number of sensors in the glove
        NUM_GESTURES: 
            Number of gestures to be training inside the training
            targets matrix.
        NUM_OUTPUTS:
            Number of joint data entries needed per sample.
        SAMPLES_PER_POSITION:
            Number of samples to take for each gesture.
        NUM_SAMPLES:
            Total number of samples to be taken.
        is_calibrating:
            True when the data is still being captured for calibration.
        is_complete:
            True when the training is complete.
        gesture_index:
            Represents the gesture currently being calibrated.
        repetition_index:
            The number of reps done for the current gesture.
        sample_index:
            The total number of samples taken.
        inputs:
            Numpy array used to contain input data.
        targets:
            Numpy array used to contain target data.
        prev_input:
            Numpy array used to track the previously processed input data.
        mtheta:
            List containing the regression line coefficients and intercepts.
        curr_target:
            Numpy array used to track the target angles for the input currently
            being processed. 
    """

    def __init__(self):
        """Constructor for a TrainingData instance"""

        # constants
        self.SOLVER = SolveLeastSquares()
        self.TRAINING_TARGETS = np.array([
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
        self.NUM_SENSORS = 7
        self.NUM_GESTURES, self.NUM_OUTPUTS = self.TRAINING_TARGETS.shape
        self.SAMPLES_PER_POSITION = 100
        self.NUM_SAMPLES = self.SAMPLES_PER_POSITION * self.NUM_GESTURES

        # booleans
        self.is_calibrating = True
        self.is_complete = False

        # indices
        self.gesture_index = 0
        self.repetition_index = 0
        self.sample_index = 0

        self.inputs = np.zeros((self.NUM_SAMPLES, self.NUM_SENSORS + 1))
        self.targets = np.zeros((self.NUM_SAMPLES, self.NUM_OUTPUTS))

        self.prev_input = np.zeros(self.NUM_SENSORS)
        self.mtheta = []
        self.curr_target = self.TRAINING_TARGETS[0]

    def get_training_data(self) -> np.ndarray:
        """Getter for the training targets

        Retrieves the desired angle data for each joint for each gesture
        to be trained.

        Returns:
            A numpy array of integers where each row represents one gesture
            and each row contains the target angles for each joint.
        """

        return self.TRAINING_TARGETS

    def update_sample(self, curr_input: np.ndarray) -> None:
        """Updates self.inputs with a new input.

        Takes in a new vector containing capacitance data, then updates the
        inputs matrix and increments the required indices. 

        Args:
            curr_input: 
                a vector of integers containing 1 new sample of capacitance
                data.
        """
        
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
                self.repetition_index += 1
                if self.repetition_index >= self.SAMPLES_PER_POSITION:
                    # Update to new position
                    self._update_gesture()
                    # Reset Flags
                    self.is_calibrating = False
                    # Reset counters
                    self.repetition_index = 0
        
    def _update_gesture(self):
        """Updates self.inputs with a new input.

        Takes in a new vector containing capacitance data, then updates the
        inputs matrix and increments the required indices. 
        """

        if self.is_complete:
            return

        self.gesture_index += 1
        
        if self.gesture_index < self.NUM_GESTURES:
            self.curr_target = self.TRAINING_TARGETS[self.gesture_index]
            print('next position in 5 seconds')

        else:
            print('Done recording')
            self.is_complete = True
            self.mtheta = self.SOLVER.solve(self.inputs, self.targets)


    

    


