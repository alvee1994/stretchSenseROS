"""Handles the models used to convert capacitance into angle data."""

import os
import time
import numpy as np
import pandas as pd
from peripheral import stretchsense_peripheral as ssp
from typing import List

class Model:
    """Encapsulates a training model."""

    def __init__(self, peripheral: ssp.StretchSensePeripheral):
        self._peripheral: ssp.StretchSensePeripheral = peripheral
        self._theta: np.ndarray

    def find_model(self) -> bool:
        """Checks if a default theta file exists.

        Checks if the defeault theta file path in self.peripheral has a valid
        file. If it does not, or the user wants to calibrate again, return
        False. Otherwise, load the theta file, update self.theta, then return
        True.

        Returns:
            False if calibration is required, and
            True otherwise.
        """
        if os.path.isfile(self._peripheral.get_default_theta_path()):
            cal = input('Found an old model file. \nCalibrate again? Y/N ')
            if cal in ['Y', 'y', 'yes', 'Yes']:
                return False
            else:
                self._load_theta()
                return True
        else:
            print('Getting ready to Calibrate in 5 secs')
            time.sleep(5)
            return False

    def _load_theta(self) -> None:
        """Loads data from the default file to mtheta."""

        default_theta = pd.read_csv(self._peripheral.get_default_theta_path(),
                                    sep=',',
                                    header=None)
        self._theta = default_theta.values

    def apply_transformation(self,
                             input_data: np.ndarray) -> np.ndarray:
        """Transforms capacitance data into the angle of each joint.

        Takes in capacitance data and theta values, then uses matrix
        multiplication to output the angle of each joint.

        Args:
            input_data: 
                a numpy array of integers representing the capacitance data
                captured by the glove.
            theta: 
                a list of integers representing the coefficients and intercepts
                of the regression line.

        Returns:
            A numpy array containing the angles of each joint.
        """

        result = np.zeros(len(self._theta))
        cleaned_input = np.ones((len(input_data), 4))

        for i in range(len(self._theta)): 
            idx = np.nonzero(self._peripheral.ACTIVE_SENSORS[i])
            filtered_input = [x for x in input_data[idx]]
            for n in range(len(filtered_input)):
                cleaned_input[i][n] = filtered_input[n]

            result[i] = np.dot(self._theta[i], cleaned_input[i])

        return result

    def get_num_sensors(self) -> int:
        """Getter for the number of sensors of the connected Peripheral."""
        return self._peripheral.NUM_SENSORS

    def get_active_sensors(self) -> np.ndarray:
        """Getter for the active sensors from the Peripheral."""
        return self._peripheral.ACTIVE_SENSORS

    def update_theta(self, new_theta: np.ndarray) -> None:
        """Setter for self.theta."""
        self._theta = new_theta

    def save_theta(self, filepath: str) -> None:
        """Saves the theta values to the given filepath.
        
        Saves self.theta as a new csv file in the given filepath.

        Args:
            filepath:
                The director where the new theta csv file is to be stored.
        """
        df = pd.DataFrame(self._theta)
        df.to_csv(filepath, index=False, header=False)
        print("saved new model")