import cv2
import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt


def getTriangleArea(tp):
    return (
        abs(
            tp[0][0] * (tp[1][1] - tp[2][1])
            + tp[1][0] * (tp[2][1] - tp[0][1])
            + tp[2][0] * (tp[0][1] - tp[1][1])
        )
        / 2
    )


def dotSegment(points, Frm, scale=2, color=(0, 255, 255)):
    tri = Delaunay(points)
    area = np.zeros(len(tri.simplices))
    X = (points * scale).astype(int)

    for i, simplex in enumerate(tri.simplices):
        cv2.polylines(Frm, np.array([X[simplex]]), True, color, 2, cv2.LINE_AA)
        area[i] = getTriangleArea(points[simplex])
        # cv2.imshow("Frm_temp", Frm)
        # cv2.waitKey(0)

    return tri, area, Frm


def drawSegment(
    points, tri, dotPair, Frm, scale=2, color=(255, 0, 255), area_diff=None
):
    area = np.zeros(len(tri.simplices))
    X = (points * scale).astype(int)
    Frm_temp = Frm.copy()

    for i, simplex in enumerate(tri.simplices):
        simplex_temp = np.zeros_like(simplex)
        for j in range(3):
            simplex_temp[j] = np.argmax(dotPair[simplex[j]][:])
        cv2.polylines(
            Frm_temp,
            np.array([X[simplex_temp]]),
            True,
            color,
            2,
            cv2.LINE_AA,
        )
        # cv2.imshow("Frm_temp", Frm_temp)
        # print("in A:", simplex, "in B:", simplex_temp)
        # cv2.waitKey(0)
        area[i] = getTriangleArea(points[simplex_temp])
    return tri, area, Frm_temp


def drawArea(points, tri, dotPair, area_diff, Frm, scale=2):
    """
    Draw the area of each triangle
    :param points: the points of the mesh
    :param tri: the triangulation of the mesh
    :param dotPair: the dotPair of the mesh
    :param Frm: the frame to draw on
    :param scale: scale the points to fit the frame
    :return: Frm_add with area
    """
    X = (points * scale).astype(int)
    Frm_temp = Frm.copy()

    for i, simplex in enumerate(tri.simplices):
        simplex_temp = np.zeros_like(simplex)
        for j in range(3):
            simplex_temp[j] = np.argmax(dotPair[simplex[j]][:])
        color = np.clip((255 - area_diff[i] * 127, 0, area_diff[i] * 127), 0, 255)
        color = (int(color[0]), int(color[1]), int(color[2]))
        cv2.fillPoly(Frm_temp, np.array([X[simplex_temp]]), tuple(color))
    Frm_add = cv2.addWeighted(Frm, 0.2, Frm_temp, 0.8, 0)
    return Frm_add


def pltDeform(points, tri, area, area_diff=None):
    plt.cla()

    plt.triplot(points[:, 0], points[:, 1], tri.simplices)
    plt.plot(points[:, 0], points[:, 1], "o")
    for i in range(len(points)):
        plt.text(points[i][0] + 1, points[i][1] - 2, str(i), fontdict={"color": "gray"})
    # print("Triangles: ", len(tri.simplices))

    for i, triangle in enumerate(tri.simplices):
        if area_diff is None:
            # print(getTriangleArea(points[triangle]))
            plt.text(
                (
                    points[triangle][0][0]
                    + points[triangle][1][0]
                    + points[triangle][2][0]
                )
                / 3
                - 3,
                (
                    points[triangle][0][1]
                    + points[triangle][1][1]
                    + points[triangle][2][1]
                )
                / 3
                - 1,
                "%.1f" % area[i],
            )
        else:
            plt.text(
                (
                    points[triangle][0][0]
                    + points[triangle][1][0]
                    + points[triangle][2][0]
                )
                / 3
                - 3,
                (
                    points[triangle][0][1]
                    + points[triangle][1][1]
                    + points[triangle][2][1]
                )
                / 3
                - 1,
                "%.1f" % area_diff[i],
            )

    return


def getAreaDiff(area_a, area_b):
    return area_b / area_a


def main():
    points = np.loadtxt("./output/saved_X.out", delimiter=" ")
    tri = Delaunay(points)
    area = np.zeros(len(tri.simplices))
    for i, simplex in enumerate(tri.simplices):
        area[i] = getTriangleArea(points[simplex])

    pltDeform(points, tri, area)
    plt.show()


if __name__ == "__main__":
    main()
