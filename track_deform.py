import cv2
import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt


def getTriangleArea(triangle):
    return (
        np.linalg.norm(np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0]))
        / 2
    )


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
    plt.show()


if __name__ == "__main__":
    main()
