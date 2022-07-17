import numpy as np
from global_params import *
from matplotlib import pyplot as plt


def edgeVisualize():
    pass


def edgeDetection(tri, area, area_diff):

    # Method 1: Linear Regression
    # triSelect = np.zeros(len(tri)).astype(bool)
    # for i, area in enumerate(area_diff):
    #     triSelect[i] = area > area_threshold and area < ad_upper and area > ad_lower

    triSelect = area > area_threshold and area < ad_upper and area > ad_lower
    for i, area in enumerate(area_diff):
        if triSelect:
            pass
    # End Method 1

    # Method 2: Block by Block

    # End Method 2


def main():
    tri = np.loadtxt("output/saved_tri.out", delimiter=",")
    area = np.loadtxt("output/saved_area.out", delimiter=",")
    area_diff = np.loadtxt("output/saved_area_diff.out", delimiter=",")
    edgeDetection(tri, area, area_diff)
    pass


if __name__ == "__main__":
    main()
