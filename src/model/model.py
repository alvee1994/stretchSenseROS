"""Handles the models used to convert capacitance into angle data."""

import os
import time
import numpy as np
import pandas as pd
from peripheral import stretchsense_peripheral as ssp
from user import user

class Model:
    """Encapsulates a training model.
    
    This class handles the conversion from a particular peripheral's
    capacitance data to joint angle data for publishing.

    Args:
        peripheral:
            The peripheral the model is calibrated for.
        user:
            The user the model is calibrated for
        pkg_directory:
            A string representing the path to the working directory
    """

    def __init__(self,
                 peripheral: ssp.StretchSensePeripheral,
                 user: user.User,
                 pkg_directory: str):
        # The peripheral and user the model is calibrated for
        self._peripheral: ssp.StretchSensePeripheral = peripheral
        self._user: user.User = user

        # The path to the working directory
        self._pkg_directory: str = pkg_directory

        # The theta values used for conversion
        self._theta: np.ndarray

    def _get_theta_path(self) -> str:
        """Gets the path to the theta file for the peripheral and user."""

        filename = f"theta_{self._user.name}_{self._peripheral.SIDE}.csv"
        return self._pkg_directory + f"/src/data/{filename}"

    def find_model(self) -> bool:
        """Checks if a default theta file exists.

        Checks if the default theta file path in self.peripheral has a valid
        file. If it does not, or the user wants to calibrate again, return
        False. Otherwise, load the theta file, update self.theta, then return
        True.

        Returns:
            False if calibration is required, and
            True otherwise.
        """

        if os.path.isfile(self._get_theta_path()):
            # If the peripheral's default theta filepath is valid

            # Prompt user if they want to recalibrate
            cal = input('Found an old model file. \nCalibrate again? Y/N ')
            if cal in ['Y', 'y', 'yes', 'Yes']:
                # If calibration is required
                return False
            else:
                # If calibration not required

                # Load the theta values from the file path
                self._load_theta()
                return True
        else:
            # If the filepath is invalid (i.e. no default theta)
            print('Getting ready to Calibrate in 5 secs')
            time.sleep(5)
            return False

    def _load_theta(self) -> None:
        """Loads data from the default file to self._theta."""

        # Gets the data from the csv
        default_theta = pd.read_csv(self._get_theta_path(),
                                    sep=',',
                                    header=None)
        
        # Update local theta
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

        # Preparing numpy arrays to contain the result and input
        result = np.zeros(len(self._theta))
        cleaned_input = np.ones((len(input_data), 4))

        # Iterate through each theta row
        for i in range(len(self._theta)):
            # Filter away inactive sensors
            idx = np.nonzero(self._peripheral.ACTIVE_SENSORS[i])
            filtered_input = [x for x in input_data[idx]]

            # Iterate through samples
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

    def save_theta(self) -> None:
        """Saves the theta values to the given filepath.
        
        Saves self.theta as a new csv file in the given filepath.
        """

        # Convert list of theta values to a pandas DataFrame
        df = pd.DataFrame(self._theta)

        # Convert DataFrame to a csv file stored in the given filepath
        df.to_csv(self._get_theta_path(),
                  index=False,
                  header=False)
        print(f"saved new model at {self._get_theta_path()}")