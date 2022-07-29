import cv2
import numpy as np
from global_params import *
from sklearn.cluster import DBSCAN
from matplotlib import pyplot as plt


def edgeVisualize(frm, result, method=1, scale=1):
    if method == 1:
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

    elif method == 2:
        frm_result = frm.copy()
        for line in result:
            x1 = int(0 * scale)
            y1 = int(int(np.polyval(line, 0)) * scale)
            x2 = int(frm.shape[0] * scale)
            y2 = int(int(np.polyval(line, frm.shape[0])) * scale)
            frm_result = cv2.line(
                frm_result,
                (x1, y1),
                (x2, y2),
                (
                    np.random.randint(0, 255),
                    np.random.randint(0, 255),
                    np.random.randint(0, 255),
                ),
                2,
                cv2.LINE_AA,
            )
        return frm_result


def edgeDetection(tri, points, dotPair, area, area_diff, frm, method=1, deg=1, scale=2):
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

    if len(triEdge) < deg + 2:
        return None, frm_result

    if method == 1:
        # Method 1: Linear Fit
        result, result_cov = np.polyfit(
            triCenter[:, 0], triCenter[:, 1], 1, full=False, cov=True
        )  #    np.polynomial.polynomial.Polynomial.fit()

        return result, frm_result
        # End Method 1
    elif method == 2:
        # Method 2: Line Clustering
        model = DBSCAN(eps=100, min_samples=4)
        yhat = model.fit_predict(triCenter)
        clusters = np.unique(yhat)
        lines = []

        for cluster in clusters:
            row = np.transpose(np.where(yhat == cluster))
            centers = triCenter[row, :]
            centers = np.reshape(centers, (np.shape(row)[0], 2))

            colors = (
                np.random.randint(100, 255),
                np.random.randint(100, 255),
                np.random.randint(100, 255),
            )
            for center in centers:
                cv2.circle(
                    frm_result,
                    (center[0].astype(int), center[1].astype(int)),
                    3,
                    colors,
                    -1,
                    cv2.LINE_AA,
                )

            if np.shape(row)[0] < deg + 2:
                continue
            lines.append(np.polyfit(centers[:, 0], centers[:, 1], 1))
        # Line Combining

        lines = np.array(lines)

        return lines, frm_result
        # End Method 2


def main():

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
