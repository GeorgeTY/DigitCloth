from functools import partial
import matplotlib.pyplot as plt
from pycpd import DeformableRegistration
import numpy as np
from global_params import *


def visualize(iteration, error, X, Y, ax):
    plt.cla()
    ax.scatter(X[:, 0], X[:, 1], color="red", label="Target")
    ax.scatter(Y[:, 0], Y[:, 1], color="blue", label="Source")
    plt.text(
        0.87,
        0.92,
        "Iteration: {:d}".format(iteration),
        horizontalalignment="center",
        verticalalignment="center",
        transform=ax.transAxes,
        fontsize="x-large",
    )
    ax.legend(loc="upper left", fontsize="x-large")
    plt.draw()
    plt.pause(0.001)


def getRegParam(self):
    return self.G, self.W, self.P


## Add P output to existing function
DeformableRegistration.get_registration_parameters = getRegParam


def main():
    X = np.loadtxt("output/saved_X.out")
    Y = np.loadtxt("output/saved_Y.out")

    fig = plt.figure()
    fig.add_axes([0, 0, 1, 1])
    callback = partial(visualize, ax=fig.axes[0])

    reg = DeformableRegistration(**{"X": X, "Y": Y}, alpha=alpha, beta=beta)
    TY, (G, W, P) = reg.register(callback)
    np.savetxt("./output/saved_P.out", P * 100, delimiter=",", fmt="%d")
    # print(TY)

    fig.axes[0].scatter(TY[:, 0], TY[:, 1], color="green", label="Target")
    plt.show()


if __name__ == "__main__":
    main()
