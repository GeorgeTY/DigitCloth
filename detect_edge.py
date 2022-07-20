from turtle import shape
import cv2
import numpy as np
from sympy import re
from global_params import *
from matplotlib import pyplot as plt


def edgeVisualize(frm, result):
    frm_result = frm.copy()
    frm_result = cv2.line(
        frm_result,
        (0, np.polyval(result, 0)),
        (frm.shape[0], np.polyval(result, frm.shape[0])),
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return frm_result


def edgeDetection(tri, points, area, area_diff):

    triSelect = area > area_threshold
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
                and (
                    area_diff[i] > ad_upper and area_diff[tri.neighbor[i][j]] < ad_lower
                )
                or area_diff[i] / area_diff[tri.neighbor[i][j]] > ad_ratio
            ):
                triEdge.append(
                    tuple(
                        tri.simplex[i][np.array(tuple(x for x in range(3) if x != j))]
                    )
                )
    triEdge = np.array(triEdge)

    for i, edge in enumerate(triEdge):
        triCenter.append(
            tuple(
                (points[edge[0]][0] + points[edge[1]][0]) / 2,
                (points[edge[0]][1] + points[edge[1]][1]) / 2,
            )
        )
    triCenter = np.array(triCenter)

    result = np.polyfit(triCenter[:, 0], triCenter[:, 1], 1)

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

    return result


def main():
    tri = np.loadtxt("output/saved_tri.out", delimiter=",")
    area = np.loadtxt("output/saved_area.out", delimiter=",")
    area_diff = np.loadtxt("output/saved_area_diff.out", delimiter=",")
    points = np.loadtxt("output/saved_points.out", delimiter=",")
    frm = cv2.imread("output/saved_frm.png")
    result = edgeDetection(tri, points, area, area_diff)
    frm_result = edgeVisualize(frm, result)
    cv2.imshow("frm_result", frm_result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
