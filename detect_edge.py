import cv2
import numpy as np
from global_params import *
from matplotlib import pyplot as plt


def edgeVisualize(frm, result, scale=1):
    frm_result = frm.copy()
    x1 = int(0 * scale)
    y1 = int(int(np.polyval(result, 0)) * scale)
    x2 = int(frm.shape[0] * scale)
    y2 = int(int(np.polyval(result, frm.shape[0])) * scale)
    frm_result = cv2.line(
        frm_result,
        (x1, y1),
        (x2, y2),
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return frm_result


def edgeDetection(tri, points, dotPair, area, area_diff, frm, deg=1, scale=2):
    frm_result = frm.copy()
    triSelect = area > area_threshold
    triCenter = []
    triEdge = []
    # looking for neighbors edges that has descending area difference
    for i in range(len(tri.simplices)):
        for j in range(3):
            if (
                tri.neighbors[i][j] != -1
                and triSelect[i]
                and triSelect[tri.neighbors[i][j]]
                # Descending area difference
                and (
                    area_diff[i] > ad_upper
                    and area_diff[tri.neighbors[i][j]] < ad_lower
                    or area_diff[i] / area_diff[tri.neighbors[i][j]] > ad_ratio
                )
            ):
                triEdge.append(
                    tuple(
                        tri.simplices[i][np.array(tuple(x for x in range(3) if x != j))]
                    )
                )
    triEdge = np.array(triEdge)
    if len(triEdge) < deg + 2:
        return None, None

    pos = (points * scale).astype(int)
    for i, edge in enumerate(triEdge):
        edge_temp = np.zeros_like(edge)
        for j in range(2):
            edge_temp[j] = np.argmax(dotPair[edge[j]][:])
        triCenter.append(
            [
                (pos[edge_temp[0]][0] + pos[edge_temp[1]][0]) / 2,
                (pos[edge_temp[0]][1] + pos[edge_temp[1]][1]) / 2,
            ]
        )
        frm_result = cv2.line(
            frm_result,
            (pos[edge_temp[0]][0].astype(int), pos[edge_temp[0]][1].astype(int)),
            (pos[edge_temp[1]][0].astype(int), pos[edge_temp[1]][1].astype(int)),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        frm_result = cv2.putText(
            frm_result,
            str(edge_temp[0]),
            (pos[edge_temp[0]][0].astype(int), pos[edge_temp[0]][1].astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        frm_result = cv2.putText(
            frm_result,
            str(edge_temp[1]),
            (pos[edge_temp[1]][0].astype(int), pos[edge_temp[1]][1].astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        # cv2.imshow("frm_result", frm_result)
        # cv2.waitKey(0)
    triCenter = np.array(triCenter)

    result, result_cov = np.polyfit(
        triCenter[:, 0], triCenter[:, 1], 1, full=False, cov=True
    )  #    np.polynomial.polynomial.Polynomial.fit()

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

    return result, frm_result


def main():

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
