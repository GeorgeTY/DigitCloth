import numpy as np
from matplotlib import pyplot as plt
from scipy import interpolate as interp


def dotInterpolation(Y, dotPair):
    """take Y and dotPair and generate dots missing in Y according to dotPair
    :param Y:
    :param dotPair:
    :return Y_new:
    """
    Y_new = Y.copy()
    for i in range(len(dotPair)):
        if dotPair[i, 0] == -1:
            continue
        else:
            x = Y[dotPair[i, 0], 0]
            y = Y[dotPair[i, 0], 1]
            x_new = Y[dotPair[i, 1], 0]
            y_new = Y[dotPair[i, 1], 1]
            x_new_list = np.linspace(x, x_new, 10)
            y_new_list = np.linspace(y, y_new, 10)
            Y_new = np.vstack((Y_new, np.column_stack((x_new_list, y_new_list))))
    return Y_new


def main():
    Y = np.loadtxt("output/saved_Y.out", delimiter=" ")
    dotPair = np.loadtxt("output/saved_dotPair.out", delimiter=",")

    Y_new = dotInterpolation(Y, dotPair)
    ## Plot difference between original and interpolated
    plt.plot(Y[:, 0], Y[:, 1], "o", label="original")
    plt.plot(Y_new[:, 0], Y_new[:, 1], "o", label="interpolated")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
