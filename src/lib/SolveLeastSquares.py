#!/usr/bin/env python3
import numpy as np
from sklearn import linear_model
from tempfile import TemporaryFile
from joblib import dump

activeSensors = np.array([
    [1, 1, 0, 1, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 1]
])

linRegCoff = TemporaryFile()

class SolveLeastSquares:

    # def Solve(self, X, yArray):
    #     y = np.zeros(len(yArray))
    #     theta = np.zeros((len(yArray[0]), 2))
    #
    #     for i in range(0, len(yArray[0])):
    #         y = self.getColumn(yArray, i)
    #         x = self.getColumn(X, i)
    #         x = np.array([x])
    #         regr = linear_model.LinearRegression()
    #         regr.fit(np.transpose(x),y)
    #         theta[i][0] = regr.coef_
    #         theta[i][1] = regr.intercept_
    #
    #     return theta

    def Solve(self, X, yArray):
        # y = np.zeros(len(yArray))
        theta = []

        for j in range(0, len(yArray[0])):
            y = self.getColumn(yArray, j)
            x = self.getColumnX(X, j)
            # x = np.array([x])
            regr = linear_model.LinearRegression()
            regr.fit(x, y)
            theta.append([s for s in regr.coef_])
            theta[j].append(regr.intercept_)

        return theta

    # def ApplyTransformation(self, inputt, theta):
    #     result = np.zeros(len(theta))
    #
    #     for i in range(len(theta)):
    #         result[i] = (inputt[i] * theta[i][0]) + theta[i][1]
    #
    #     return result

    def ApplyTransformation(self, inputt, theta):
        result = np.zeros(len(theta))
        inp = np.zeros((len(inputt), 4))

        for i in range(len(theta)):
            idx = np.nonzero(activeSensors[i])
            list = [x for x in inputt[idx]]
            for n in range(len(list)):
                inp[i][n] = list[n]

            inp[i][3] = 1
            result[i] = np.dot(theta[i], inp[i])

        return result

    def getColumn(self, Array, index):
        result = np.zeros(len(Array))

        for i in range(0, len(Array)):
            result[i] = Array[i][index]
            
        return result


    def getColumnX(self, Array, index):
        result = np.zeros((len(Array), 3))

        for i in range(len(Array)):
            idx = np.nonzero(activeSensors[index])
            list = Array[i][idx]
            for r in range(len(list)):
                result[i][r] = list[r]

        return result


