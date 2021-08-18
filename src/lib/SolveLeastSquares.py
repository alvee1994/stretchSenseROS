#!/usr/bin/env python3
import numpy as np
from sklearn import linear_model
from tempfile import TemporaryFile
from joblib import dump

activeSensors = np.array([
    [1, 1, 0, 1, 0, 0, 0, 0], # thumb
    [0, 1, 1, 1, 0, 0, 0, 0], # thumb
    [0, 0, 0, 0, 1, 0, 0, 0], # index
    [0, 0, 0, 0, 0, 1, 0, 0], # middle
    [0, 0, 0, 0, 0, 0, 1, 0], # ring
    [0, 0, 0, 0, 0, 0, 0, 1]  # pinky
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

    # def ApplyTransformation(self, inputt, theta):
    #     result = np.zeros(len(theta))
    #
    #     for i in range(len(theta)):
    #         result[i] = (inputt[i] * theta[i][0]) + theta[i][1]
    #
    #     return result

    def ApplyTransformation(self, inputt, theta):
        result = np.zeros(len(theta))
        inp = np.ones((len(inputt), 4))

        for i in range(len(theta)):
            idx = np.nonzero(activeSensors[i])
            list = [x for x in inputt[idx]]
            for n in range(len(list)):
                inp[i][n] = list[n]

            result[i] = np.dot(theta[i], inp[i])

        return result

    # def getColumn(self, Array, index):
    #     result = np.zeros(len(Array))
    #
    #     for i in range(0, len(Array)):
    #         result[i] = Array[i][index]
    #
    #     return result


    # def getColumnX(self, Array, index):
    #     result = np.zeros((len(Array), 3)) # 200, 3
    #
    #     for i in range(len(Array)):
    #         idx = np.nonzero(activeSensors[index])
    #         list = Array[i][idx]
    #         for r in range(len(list)):
    #             result[i][r] = list[r]
    #
    #     return result

# np.array([0.0, -9.532973539245603, 5.0976739742520145, 530.5248454769321,
# -8.99699712791857, 5.869142396940321, 1.6784991147593107, -693.9198822829043,
# -1.6996151234519972, 0.0, 0.0, 470.74101388401806,
# -1.2290345581775315, 0.0, 0.0, 361.92842607434767,
# -2.2188270439459608, 0.0, 0.0, 585.5169808315806,
# -1.5782019943629093, 0.0, 0.0, 458.6590864072555])


