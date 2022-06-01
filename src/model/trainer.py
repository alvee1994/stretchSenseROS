"""Classes for training the linear regression models."""

import numpy as np
from model import model, solver

class Trainer:
    """Collects data then updates Model with the calculated theta values.
    
    This class is an observer subscribed to the Application. As calibration
    takes place in the App, it calls the update_sample method of this class,
    which stores data until all the training is complete, then the data is
    passed to the Solver, which generates the theta values. Then, those values
    are used to update the Model.

    Args:
        model:
            The model to be trained by this trainer.

    Attributes:
        TRAINING_TARGETS:
            A numpy array representing the target angles for each gesture.
        is_calibrating:
            A boolean indicating whether calibration is underway.
        is_complete:
            A boolean indicating whether calibration is complete.
        gesture_index:
            An integer representing the gesture being trained currently.
    """

    def __init__(self, model: model.Model):
        # Model to be trained
        self._model = model

        # Solver used to find the linear regression line
        self._solver = solver.LeastSquaresSolver()

        # Constants
        self._SAMPLES_PER_POSITION : int = 100
        self.TRAINING_TARGETS: np.ndarray = np.array([
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
        self._NUM_GESTURES: int = self.TRAINING_TARGETS.shape[0]
        self._NUM_OUTPUTS: int = self.TRAINING_TARGETS.shape[1]
        self._NUM_SAMPLES: int = (self._SAMPLES_PER_POSITION
                                  * self._NUM_GESTURES)
        self._NUM_SENSORS: int = self._model.get_num_sensors()

        # Flags
        self.is_calibrating: bool = True
        self.is_complete: bool = False

        # indices
        self.gesture_index: int = 0
        self._repetition_index: int = 0
        self._sample_index: int = 0

        # Input and target matrices to be solved
        self._inputs: np.ndarray = np.zeros((self._NUM_SAMPLES,
                                             self._NUM_SENSORS + 1))
        self._targets: np.ndarray = np.zeros((self._NUM_SAMPLES,
                                              self._NUM_OUTPUTS))

        # Helper variables
        self._prev_input: np.ndarray = np.zeros(self._NUM_SENSORS)
        self._curr_target: np.ndarray = self.TRAINING_TARGETS[0]


    def update_sample(self, curr_input: np.ndarray) -> None:
        """Updates self.inputs with a new input.

        Takes in a new vector containing capacitance data, then updates the
        inputs matrix and increments the required indices. 

        Args:
            curr_input: 
                a vector of integers containing 1 new sample of capacitance
                data.
        """
        
        # Skip if training is already complete
        if self.is_complete:
            return

        if np.array_equal(curr_input, self._prev_input):
            # Skip if duplicate data is received
            print('no new data received\n')
            return
        else:
            # If input data is different from previous

            # Update prev
            self._prev_input = curr_input

            if self.is_calibrating:
                # If calibration is underway

                # Update the array of input samples
                self._inputs[self._sample_index][0] = 1
                self._inputs[self._sample_index][1:] = np.array(curr_input)

                # Update corresponding targets
                self._targets[self._sample_index] = self._curr_target

                # Increment pointers
                self._sample_index += 1
                self._repetition_index += 1

                if self._repetition_index >= self._SAMPLES_PER_POSITION:
                    # If enough data collected for this gesture

                    # Update to new position
                    self._update_gesture()
                    # Reset Flags
                    self.is_calibrating = False
                    # Reset counter
                    self._repetition_index = 0
        
    def _update_gesture(self) -> None:
        """Updates self.inputs with a new input.

        Takes in a new vector containing capacitance data, then updates the
        inputs matrix and increments the required indices. 
        """

        # Skip if training is already complete
        if self.is_complete:
            return

        # Increment gesture pointer
        self.gesture_index += 1
        
        if self.gesture_index < self._NUM_GESTURES:
            # If there are still gestures to be trained

            # Move on to training the next gesture
            self._curr_target = self.TRAINING_TARGETS[self.gesture_index]
            print('next position in 5 seconds')

        else:
            # If there are no more gestures to be trained

            # Print alert
            print('Done recording')

            # Update completion flag
            self.is_complete = True

            # Calculate theta values
            result = self._solver.solve(self._inputs,
                                        self._targets,
                                        self._model.get_active_sensors())

            # Update the model's theta values
            self._model.update_theta(result)
