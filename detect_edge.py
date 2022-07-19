import cv2
import numpy as np
from global_params import *
from matplotlib import pyplot as plt


def edgeVisualize(Frm, x1, y1, x2, y2):
    Frm = cv2.line(Frm, (x1, y1), (x2, y2), (0, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow("Preview", Frm)
    cv2.moveWindow("Preview", 2020, 100)
    cv2.waitKey(0)
    return


def edgeDetection(tri, area, area_diff):

    triSelect = area > area_threshold and area < ad_upper and area > ad_lower
    triCenter = []
    triEdge = []
    # looking for neighbor edges that has descending area difference
    for i in range(len(tri)):
        for j in range(3):
            if (
                tri.neighbor[i][j] != -1
                and triSelect[i]
                and triSelect[tri.neighbor[i][j]]
                # Descending area difference
                and area_diff[i] > 1
                and area_diff[tri.neighbor[i][j]] < 1
            ):
                triEdge.append(tri.simplex[i][tuple(x for x in range(3) if x != j)])
    # Method 1: Linear Regression
    # for i, area in enumerate(area_diff):
    #     if triSelect:
    #         triCenter.append(triSelect)
    # triCenter = np.array(triCenter)

    # a, b = np.polyfit(triCenter[:, 0], triCenter[:, 1], 1)
    # End Method 1

    # Method 2: Gradient Descent

    # End Method 2

    # Method 3: Block by Block

    # End Method 3

    return triEdge


def main():
    tri = np.loadtxt("output/saved_tri.out", delimiter=",")
    area = np.loadtxt("output/saved_area.out", delimiter=",")
    area_diff = np.loadtxt("output/saved_area_diff.out", delimiter=",")
    Frm = cv2.imread("output/saved_Frm.png")
    edgeDetection(tri, area, area_diff)


if __name__ == "__main__":
    main()
