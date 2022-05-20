#!/usr/bin/env python3
"""Module for solving least squares. """
from typing import List
import numpy as np
from sklearn import linear_model




class SolveLeastSquares:
    """This class is flawed and needs to be changed.

    TODO: ensure the Single Responsibility principle is obeyed.

    Attributes:
        ACTIVE_SENSORS:
            a numpy array representing the sensors that correspond to
            each digit.
        TODO: change to boolean array
    """

    ACTIVE_SENSORS = np.array([
        [1, 1, 0, 1, 0, 0, 0, 0], # thumb
        [0, 1, 1, 1, 0, 0, 0, 0], # thumb
        [0, 0, 0, 0, 1, 0, 0, 0], # index
        [0, 0, 0, 0, 0, 1, 0, 0], # middle
        [0, 0, 0, 0, 0, 0, 1, 0], # ring
        [0, 0, 0, 0, 0, 0, 0, 1]  # pinky
    ])

    def solve(self,
              input_data: np.ndarray,
              target_data: np.ndarray) -> np.ndarray:
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

            # Cleaning the input data
            idx = np.nonzero(SolveLeastSquares.ACTIVE_SENSORS[index])
            cleaned_input = input_data[:,idx].flatten().reshape(
                    input_data.shape[0], -1)
            cols = cleaned_input.shape[-1]
            cleaned_input = np.pad(cleaned_input,
                                   ((0,0),(0,3-cols)),
                                   'constant')

            # Performing linear regression on the cleaned input and target data
            regr = linear_model.LinearRegression()
            regr.fit(cleaned_input, targets)

            # Append results to output list
            theta.append([s for s in regr.coef_])
            theta[index].append(regr.intercept_)

        print('DONE training')
        return theta

    def apply_transformation(self,
                             input_data: np.ndarray,
                             theta: List) -> np.ndarray:
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

        result = np.zeros(len(theta))
        cleaned_input = np.ones((len(input_data), 4))

        for i in range(len(theta)): 
            idx = np.nonzero(SolveLeastSquares.ACTIVE_SENSORS[i])
            filtered_input = [x for x in input_data[idx]]
            for n in range(len(filtered_input)):
                cleaned_input[i][n] = filtered_input[n]

            result[i] = np.dot(theta[i], cleaned_input[i])

        return result




