import cv2
import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt


def getTriangleArea(tp):
    return (
        tp[0][0] * (tp[1][1] - tp[2][1])
        + tp[1][0] * (tp[2][1] - tp[0][1])
        + tp[2][0] * (tp[0][1] - tp[1][1])
    ) / 2


def dotSegment(points, Frm, scale=2, color=(0, 255, 255)):
    tri = Delaunay(points)
    X = (points * scale).astype(int)

    for simplex in tri.simplices:
        cv2.polylines(
            Frm,
            np.array([X[simplex]]),
            True,
            color,
            2,
        )
        # simplex = np.append(simplex, simplex[0])
        # print(simplex)
        # for i in range(len(simplex) - 1):
        #     cv2.line(
        #         Frm,
        #         (int(X[simplex[i]][0] * 2), int(X[simplex[i]][1] * 2)),
        #         (int(X[simplex[i + 1]][0] * 2), int(X[simplex[i + 1]][1] * 2)),
        #         (0, 255, 255),
        #         2,
        #     )

    return tri, Frm


def main():
    points = np.loadtxt("./output/saved_X.out", delimiter=" ")
    tri = Delaunay(points)

    plt.triplot(points[:, 0], points[:, 1], tri.simplices)
    plt.plot(points[:, 0], points[:, 1], "o")
    for i in range(len(points)):
        plt.text(points[i][0] + 1, points[i][1] - 2, str(i), fontdict={"color": "gray"})
    print(len(tri.simplices))

    for triangle in tri.simplices:
        # print(getTriangleArea(points[triangle]))
        plt.text(
            (points[triangle][0][0] + points[triangle][1][0] + points[triangle][2][0])
            / 3
            - 3,
            (points[triangle][0][1] + points[triangle][1][1] + points[triangle][2][1])
            / 3
            - 1,
            "%.1f" % getTriangleArea(points[triangle]),
        )

    plt.show()


if __name__ == "__main__":
    main()
