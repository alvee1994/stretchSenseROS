#!/usr/bin/env python3
import numpy as np
from sklearn import linear_model
from tempfile import TemporaryFile

activeSensors = np.array([
    [1, 1, 0, 1, 0, 0, 0, 0], # thumb
    [0, 1, 1, 1, 0, 0, 0, 0], # thumb
    [0, 0, 0, 0, 1, 0, 0, 0], # index
    [0, 0, 0, 0, 0, 1, 0, 0], # middle
    [0, 0, 0, 0, 0, 0, 1, 0], # ring
    [0, 0, 0, 0, 0, 0, 0, 1]  # pinky
])

# linRegCoff = TemporaryFile()

class SolveLeastSquares:

    def Solve(self, X, yArray):
        # y = np.zeros(len(yArray))
        theta = []

        t_yArray = yArray.transpose()

        print(f'shape of yArray is {yArray.shape}')

        for index, _ in enumerate(t_yArray):
            y = t_yArray[index] # column of a single finger
            idx = np.nonzero(activeSensors[index])
            X_copy = X[:,idx].flatten().reshape(X.shape[0], -1)
            print(f'the idx is {idx}\n')
            cols = X_copy.shape[-1]
            X_copy = np.pad(X_copy, ((0,0),(0,3-cols)), 'constant')
            print(f'\n######\n\n{X_copy}\n\n######\n')
            regr = linear_model.LinearRegression()
            regr.fit(X_copy, y)
            theta.append([s for s in regr.coef_])
            theta[index].append(regr.intercept_)

        print('DONE training')
        return theta

    def ApplyTransformation(self, inputt, theta):
        result = np.zeros(len(theta))
        inp = np.ones((len(inputt), 4))

        for i in range(len(theta)):
            idx = np.nonzero(activeSensors[i])
            list_ = [x for x in inputt[idx]]
            for n in range(len(list_)):
                inp[i][n] = list_[n]

            result[i] = np.dot(theta[i], inp[i])

        return result




