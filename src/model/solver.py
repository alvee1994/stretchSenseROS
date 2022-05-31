"""Solvers used to find the least squares regression lines for the data."""

import numpy as np
from sklearn import linear_model

class LeastSquaresSolver:
    """Uses scikit-learn's linear_model module to solve the regression line."""

    def solve(self,
              input_data: np.ndarray,
              target_data: np.ndarray,
              active_sensors: np.ndarray) -> np.ndarray:
        """Finds and returns the least squares regression line.

        For each finger, takes the capacitance data, filters out the
        unnecessary sensors, then calculates the linear regression line with
        respect to the given target data.

        Args:
            input_data: 
                a numpy array of integers. Each row is a unique sample
                and each sample contains the capacitance data from the glove
            target_data: 
                a numpy array of integers. Each row is a sample
                corresponding to input_data and contains the target angles per
                joint

        Returns:
            A list containing the coefficients and intercept of the calculated
            regression line.
        """
        
        theta = []
        target_data_transposed = target_data.transpose()

        # Iterating through data for each finger
        for index, targets in enumerate(target_data_transposed):
            # Cleaning input matrix
            cleaned_input = self._clean_input(input_data,
                                              index,
                                              active_sensors)

            # Performing linear regression on the cleaned input and target data
            regr = linear_model.LinearRegression()
            regr.fit(cleaned_input, targets)

            # Append results to output list
            theta.append([s for s in regr.coef_])
            theta[index].append(regr.intercept_)

        print('DONE training')
        return np.array(theta)

    def _clean_input(self,
                     input_data: np.ndarray,
                     index: int,
                     active_sensors: np.ndarray) -> np.ndarray:
        """Cleans the input data for regression.
        
        Filters away the non active sensors, then pads zero columns such that
        the output always has 3 columns.

        Args:
            input_data:
                A numpy array containing the input data for training.
            index:
                An integer representing the gesture currently being trained.
            active_sensors:
                A numpy array representing the peripheral's active sensors.
        """

        # Filter out unnecessary sensors and reshape
        idx = np.nonzero(active_sensors[index])
        cleaned_input = input_data[:,idx].flatten().reshape(
                input_data.shape[0], -1)
        cols = cleaned_input.shape[-1]

        # Pad with zeroes to ensure output always has 3 columns
        return np.pad(cleaned_input,
                      ((0,0),(0,3-cols)),
                      'constant')