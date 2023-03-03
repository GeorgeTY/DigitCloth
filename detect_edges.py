import cv2
import time
import numpy as np
from global_params import *
from collections import deque
from sklearn.cluster import DBSCAN


class Edges:
    sample_length = 128
    edges_queue = deque(maxlen=sample_length)
    clusters_queue = deque(maxlen=sample_length)

    def __init__(self) -> None:
        self.model = DBSCAN(eps=30, min_samples=6)
        self.is_timing = False
        self.is_visualize = True

    def detect(self, keypoints, tris, area, area_diff):
        tic = time.time()

        tris_selected = area > area_threshold
        # Filter triangles by angle (if the largest angle is too large, it is not a acceptable triangle)
        for i in range(len(tris_selected)):
            if tris_selected[i]:
                tris_edges_length = [
                    np.linalg.norm(keypoints[tris.simplices[i][0]] - keypoints[tris.simplices[i][1]]),
                    np.linalg.norm(keypoints[tris.simplices[i][1]] - keypoints[tris.simplices[i][2]]),
                    np.linalg.norm(keypoints[tris.simplices[i][2]] - keypoints[tris.simplices[i][0]]),
                ]
                tris_edges_length.sort()
                if (tris_edges_length[0] ** 2 + tris_edges_length[1] ** 2 - tris_edges_length[2] ** 2) / (
                    2 * tris_edges_length[0] * tris_edges_length[1]
                ) < np.cos(np.pi * angle_threshold):
                    tris_selected[i] = False
        # Looking for neighbors self.edges that has descending area difference
        tris_edges = []
        self.centers = []
        for i in range(len(tris.simplices)):
            for j in range(3):
                if (
                    tris.neighbors[i][j] != -1
                    and tris_selected[i]
                    and tris_selected[tris.neighbors[i][j]]
                    and (
                        area_diff[i] > ad_upper
                        and area_diff[tris.neighbors[i][j]] < ad_lower
                        or area_diff[i] / area_diff[tris.neighbors[i][j]] > ad_ratio
                    )
                ):
                    edge = tuple(tris.simplices[i][np.array(tuple(x for x in range(3) if x != j))])
                    tris_edges.append(edge)
                    center = tuple(
                        (
                            (keypoints[edge[0]][0] + keypoints[edge[1]][0]) / 2,
                            (keypoints[edge[0]][1] + keypoints[edge[1]][1]) / 2,
                        )
                    )
                    self.centers.append(center)
        self.tris_edges = np.array(tris_edges)
        self.centers = np.array(self.centers)
        # Clustering self.edges with DBSCAN
        self.edges = []
        if len(self.centers) > 0:
            y_hat = self.model.fit_predict(self.centers)
            self.clusters = np.unique(y_hat)
            for cluster in self.clusters:
                row = np.transpose((np.where(y_hat == cluster)))
                line_centers = self.centers[row, :]
                line_centers = np.reshape(line_centers, (np.shape(row)[0], 2))
                if np.shape(row)[0] < 3:
                    continue
                self.edges.append(np.polyfit(line_centers[:, 0], line_centers[:, 1], 1))

        print("Edges: Detect Time: ", time.time() - tic) if self.is_timing else None

    def visualize(self, frame, keypoints):
        self.frame_curr_with_edges = frame.copy()
        if len(self.centers) > 0:
            for center in self.centers:
                self.frame_curr_with_edges = cv2.circle(
                    self.frame_curr_with_edges,
                    (int(center[0]), int(center[1])),
                    2,
                    (255, 255, 0),
                    -1,
                    cv2.LINE_AA,
                )
        if len(self.edges) > 0:
            for edge in self.edges:
                x1 = 0
                x2 = frame.shape[0]
                y1 = int(np.polyval(edge, 0))
                y2 = int(np.polyval(edge, frame.shape[0]))
                self.frame_curr_with_edges = cv2.line(
                    self.frame_curr_with_edges,
                    (x1, y1),
                    (x2, y2),
                    (
                        np.random.randint(100, 255),
                        np.random.randint(100, 255),
                        np.random.randint(100, 255),
                    ),
                    2,
                    cv2.LINE_AA,
                )
        cv2.imshow("Edges", self.frame_curr_with_edges)
        cv2.moveWindow("Edges", 1000, 100)

    def update(self, frame, visualized_frame, keypoints, tris, area, area_diff):
        self.detect(keypoints, tris, area, area_diff)
        if self.is_visualize:
            self.visualize(frame, keypoints)


def main():
    edges = Edges()


if __name__ == "__main__":
    main()
