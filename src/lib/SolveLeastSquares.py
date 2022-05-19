#!/usr/bin/env python3
import numpy as np
from sklearn import linear_model

ACTIVE_SENSORS = np.array([
    [1, 1, 0, 1, 0, 0, 0, 0], # thumb
    [0, 1, 1, 1, 0, 0, 0, 0], # thumb
    [0, 0, 0, 0, 1, 0, 0, 0], # index
    [0, 0, 0, 0, 0, 1, 0, 0], # middle
    [0, 0, 0, 0, 0, 0, 1, 0], # ring
    [0, 0, 0, 0, 0, 0, 0, 1]  # pinky
])


class SolveLeastSquares:
    def solve(self, input_data, target_data):
        theta = []

        target_data_transposed = target_data.transpose()

        for index, targets in enumerate(target_data_transposed):

            # Cleaning the input data
            idx = np.nonzero(ACTIVE_SENSORS[index])
            cleaned_input = input_data[:,idx].flatten().reshape(input_data.shape[0], -1)
            cols = cleaned_input.shape[-1]
            cleaned_input = np.pad(cleaned_input, ((0,0),(0,3-cols)), 'constant')

            # Performing linear regression on the cleaned input and target data
            regr = linear_model.LinearRegression()
            regr.fit(cleaned_input, targets)

            # Append results to output list
            theta.append([s for s in regr.coef_])
            theta[index].append(regr.intercept_)

        print('DONE training')
        return theta

    def apply_transformation(self, input_data, theta):
        result = np.zeros(len(theta))
        cleaned_input = np.ones((len(input_data), 4))

        for i in range(len(theta)): 
            idx = np.nonzero(ACTIVE_SENSORS[i])
            filtered_input = [x for x in input_data[idx]]
            for n in range(len(filtered_input)):
                cleaned_input[i][n] = filtered_input[n]

            result[i] = np.dot(theta[i], cleaned_input[i])

        return result




