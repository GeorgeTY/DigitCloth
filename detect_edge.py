import cv2
import numpy as np
from global_params import *
from sklearn.cluster import DBSCAN
from matplotlib import pyplot as plt
from numpy.random import default_rng
from copy import copy

rng = default_rng()


class RANSAC:
    def __init__(self, n=10, k=100, t=0.05, d=10, model=None, loss=None, metric=None):
        self.n = n  # `n`: Minimum number of data points to estimate parameters
        self.k = k  # `k`: Maximum iterations allowed
        self.t = t  # `t`: Threshold value to determine if points are fit well
        self.d = (
            d  # `d`: Number of close data points required to assert model fits well
        )
        self.model = model  # `model`: class implementing `fit` and `predict`
        self.loss = (
            loss  # `loss`: function of `y_true` and `y_pred` that returns a vector
        )
        self.metric = (
            metric  # `metric`: function of `y_true` and `y_pred` and returns a float
        )
        self.best_fit = None
        self.best_error = np.inf

    def fit(self, X, y):

        for _ in range(self.k):
            ids = rng.permutation(X.shape[0])

            maybe_inliers = ids[: self.n]
            maybe_model = copy(self.model).fit(
                X[maybe_inliers], y[maybe_inliers])

            thresholded = (
                self.loss(y[ids][self.n:],
                          maybe_model.predict(X[ids][self.n:]))
                < self.t
            )

            inlier_ids = ids[self.n:][np.flatnonzero(thresholded).flatten()]

            if inlier_ids.size > self.d:
                inlier_points = np.hstack([maybe_inliers, inlier_ids])
                better_model = copy(self.model).fit(
                    X[inlier_points], y[inlier_points])

                this_error = self.metric(
                    y[inlier_points], better_model.predict(X[inlier_points])
                )

                if this_error < self.best_error:
                    self.best_error = this_error
                    self.best_fit = maybe_model

        return self

    def predict(self, X):
        return self.best_fit.predict(X)


def square_error_loss(y_true, y_pred):
    return (y_true - y_pred) ** 2


def mean_square_error(y_true, y_pred):
    return np.sum(square_error_loss(y_true, y_pred)) / y_true.shape[0]


class LinearRegressor:
    def __init__(self):
        self.params = None

    def fit(self, X: np.ndarray, y: np.ndarray):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), X])
        self.params = np.linalg.inv(X.T @ X) @ X.T @ y
        return self

    def predict(self, X: np.ndarray):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), X])
        return X @ self.params


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
    if ifGSmini:
        triSelect = area > area_threshold_gsmini
    else:
        triSelect = area > area_threshold
    # Filter triangles by angle (if the largest angle is too large, it is not a acceptable triangle)
    for i in range(len(triSelect)):
        if triSelect[i]:
            edgeLength = [
                np.linalg.norm(
                    points[tri.simplices[i][0]] - points[tri.simplices[i][1]]
                ),
                np.linalg.norm(
                    points[tri.simplices[i][1]] - points[tri.simplices[i][2]]
                ),
                np.linalg.norm(
                    points[tri.simplices[i][2]] - points[tri.simplices[i][0]]
                ),
            ]
            edgeLength.sort()
            if (edgeLength[0] ** 2 + edgeLength[1] ** 2 - edgeLength[2] ** 2) / (
                2 * edgeLength[0] * edgeLength[1]
            ) < np.cos(np.pi * angle_threshold):
                # print("suteta", edgeLength, (edgeLength[0] ** 2 + edgeLength[1]
                #   ** 2 - edgeLength[2] ** 2)/(2 * edgeLength[0] * edgeLength[1]))
                triSelect[i] = False

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
                        tri.simplices[i][np.array(
                            tuple(x for x in range(3) if x != j))]
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
            (pos[edge_temp[0]][0].astype(int),
             pos[edge_temp[0]][1].astype(int)),
            (pos[edge_temp[1]][0].astype(int),
             pos[edge_temp[1]][1].astype(int)),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        frm_result = cv2.putText(
            frm_result,
            str(edge_temp[0]),
            (pos[edge_temp[0]][0].astype(int),
             pos[edge_temp[0]][1].astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        frm_result = cv2.putText(
            frm_result,
            str(edge_temp[1]),
            (pos[edge_temp[1]][0].astype(int),
             pos[edge_temp[1]][1].astype(int)),
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
        )  # np.polynomial.polynomial.Polynomial.fit()

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
    elif method == 3:
        # Method 3: RANSAC
        regressor = RANSAC(
            model=LinearRegressor(),
            loss=square_error_loss,
            metric=mean_square_error,
            n=ransac_minInliers,
            k=ransac_iterations,
            t=ransac_threshold,
        )
        X = np.reshape(triCenter[:, 0], (np.shape(triCenter)[0], 1))
        Y = np.reshape(triCenter[:, 1], (np.shape(triCenter)[0], 1))
        regressor.fit(X, Y)
        line = np.linspace(0, frm.shape[0], frm.shape[0])
        Y = regressor.best_fit.predict(line)
        frm_result = cv2.line(
            frm_result,
            (0, Y[0].astype(int)),
            (frm.shape[1], Y[-1].astype(int)),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        result = np.polyfit(line, Y, 1)
        return result, frm_result
        # End Method 3
        return result, frm_result
        # End Method 3


def main():

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
