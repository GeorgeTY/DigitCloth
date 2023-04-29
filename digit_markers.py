import cv2
import time
import numpy as np
from collections import deque
from global_params import *
import multiprocessing as mp
from sklearn.cluster import DBSCAN
from scipy.spatial import Delaunay
from digit_device import Device
from detect_blob import setDetectionParams, dotDetection


class Markers:  # Tracking + Marker Segment + Edge Detection + Force Detection
    sample_length = 128
    dots_count = 284

    frame_queue = deque(maxlen=sample_length)
    keypoints_queue = deque(maxlen=sample_length)
    keypoints_mask_queue = deque(maxlen=sample_length)
    tris_queue = deque(maxlen=sample_length)
    area_queue = deque(maxlen=sample_length)
    area_diff_queue = deque(maxlen=sample_length)

    def __init__(self):
        self.blob_detector = cv2.SimpleBlobDetector_create(setDetectionParams())
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=1, blockSize=2)  # 100, 0.3, 7, 7
        self.lk_params = dict(
            winSize=(20, 20),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1, 0.03),  # 10, 0.03
        )
        self.model = DBSCAN(eps=30, min_samples=6)
        self.is_ros = True
        self.is_init = True
        self.is_timing = False
        self.is_visualize = True

    def __get_triangle_area(self, tp) -> float:
        return (
            abs(tp[0][0] * (tp[1][1] - tp[2][1]) + tp[1][0] * (tp[2][1] - tp[0][1]) + tp[2][0] * (tp[0][1] - tp[1][1]))
            / 2
        )

    def init(self, frame):
        self.frame_queue.clear()
        self.keypoints_queue.clear()
        self.keypoints_mask_queue.clear()
        self.track(frame, is_init=True)

    def track(self, frame):
        """Track the markers in the current frame."""
        tic = time.time()

        if len(self.frame_queue) == self.sample_length:
            self.frame_queue.popleft()
            self.keypoints_queue.popleft()
            self.keypoints_mask_queue.popleft()

        self.frame_queue.append(frame)

        keypoints, self.frame_curr_with_keypoints = dotDetection(self.blob_detector, frame)
        if self.is_init:
            # Capture the first frame and find the initial keypoints
            keypoint_count = len(keypoints)
            # (num_points, 1, 2)
            keypoints = np.array([[p.pt[0], p.pt[1]] for p in keypoints], dtype=np.float32)
            keypoints_mask = np.ones((keypoint_count,), dtype=np.int8)

            self.keypoints_queue.append(keypoints)
            self.keypoints_mask_queue.append(keypoints_mask)
            self.is_init = False
        else:
            # Optical flow
            prev_frame, next_frame = self.frame_queue[-2], self.frame_queue[-1]
            prev_keypoints = self.keypoints_queue[-1]
            prev_keypoints_mask = self.keypoints_mask_queue[-1]

            available_keypoints_indices = np.nonzero(prev_keypoints_mask)[0]
            available_prev_keypoints = prev_keypoints[available_keypoints_indices]
            available_next_keypoints = np.zeros_like(available_prev_keypoints)

            available_next_keypoints, status, _ = cv2.calcOpticalFlowPyrLK(
                prevImg=prev_frame,
                nextImg=next_frame,
                prevPts=available_prev_keypoints,
                nextPts=available_next_keypoints,
                **self.lk_params,
            )

            next_keypoints = np.zeros_like(prev_keypoints)
            next_keypoints_mask = np.zeros_like(prev_keypoints_mask)
            for i in range(len(available_next_keypoints)):
                if status[i]:
                    index = available_keypoints_indices[i]
                    next_keypoints[index] = available_next_keypoints[i]
                    next_keypoints_mask[index] = 1
            self.keypoints_queue.append(next_keypoints)
            self.keypoints_mask_queue.append(next_keypoints_mask)

        print("Markers: Update Time: ", time.time() - tic) if self.is_timing else None

    def segment(self, is_init):
        keypoints = self.keypoints_queue[-1]
        if len(keypoints) < 3:
            print("Markers: Not enough keypoints to segment.")
            return

        if is_init:
            tris = Delaunay(keypoints)
            self.tris_init = tris
        else:
            tris = self.tris_init

        area = np.zeros(len(tris.simplices))

        for i, simplex in enumerate(tris.simplices):
            area[i] = self.__get_triangle_area(keypoints[simplex])

        self.area_queue.append(area)
        self.area_diff_queue.append(self.area_queue[-1] / self.area_queue[0]) if len(self.area_queue) > 1 else None

    def edge(self):
        area_threshold = 10
        angle_threshold = 7 / 8  # * np.pi
        ad_upper = 1.2
        ad_lower = 0.8
        ad_ratio = 1.2  # TODO: find a better way to store these parameters

        keypoints = self.keypoints_queue[-1]
        tris = self.tris_init
        area = self.area_queue[-1]
        area_diff = self.area_diff_queue[-1]
        print(keypoints.shape, tris.simplices.shape, area.shape, area_diff.shape)
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

    def force(self, frame):
        # Temporary Implementation
        canny = cv2.Canny(frame, 100, 200)
        mean = cv2.meanStdDev(canny)  # TODO: ros comms
        return mean

    def visualize(self, output_queue=None):
        """Visualize the keypoints movement on the current frame."""
        """Set output_queue to None to show the visualization in place."""
        """Or set output_queue to a queue to show the visualization in main thread."""

        # packing output as a dictionary
        # example: output = {"process": "Markers", "frame_1"}

        if self.is_timing:
            tic = time.time()

        # region Visualize: Keypoints Movement
        if len(self.keypoints_queue) > 1:
            prev_frame = self.frame_queue[-2]
            prev_keypoints, curr_keypoints = self.keypoints_queue[-2], self.keypoints_queue[-1]
            curr_keypoints_mask = self.keypoints_mask_queue[-1]
            frame_curr_with_flow_vectors = cv2.addWeighted(prev_frame, 0.5, self.frame_curr_with_keypoints, 0.5, 0)

            for i, (new, old) in enumerate(zip(prev_keypoints, curr_keypoints)):
                if curr_keypoints_mask[i]:
                    a, b = new.flatten()
                    c, d = old.flatten()
                    a, b, c, d = int(a), int(b), int(c), int(d)
                    cv2.arrowedLine(
                        frame_curr_with_flow_vectors,
                        (a - (a - c) * 2, b - (b - d) * 2),
                        (c, d),
                        (124, 67, 237),  # KizunaAI Color
                        2,
                        cv2.LINE_AA,
                        tipLength=0.25,
                    )

            available_keypoints_count = np.count_nonzero(curr_keypoints_mask)
            cv2.putText(
                frame_curr_with_flow_vectors,
                str(available_keypoints_count),
                (0, 15),
                cv2.FONT_HERSHEY_COMPLEX,
                0.5,
                (255, 255, 255),
            )
        # endregion

        # region Visualize: Segmentation
        frame = self.frame_queue[-1]
        keypoints = self.keypoints_queue[-1]
        frame_curr_with_tris = frame.copy()
        keypoints_int32 = np.int32(keypoints)
        tris = self.tris_init
        area_diff = self.area_diff_queue[-1]
        for i, simplex in enumerate(tris.simplices):
            color = np.clip(
                (127 - (area_diff[i] - 1) * 2 * 255, 0, 127 + (area_diff[i] - 1) * 2 * 255),
                0,
                255,
            )
            color = (int(color[0]), int(color[1]), int(color[2]))
            cv2.fillPoly(frame_curr_with_tris, np.array([keypoints_int32[simplex]]), tuple(color))
            cv2.polylines(
                frame_curr_with_tris, np.array([keypoints_int32[simplex]]), True, (0, 255, 255), 1, cv2.LINE_AA
            )
        # endregion

        # region Visualize: Edges
        frame_curr_with_edges = frame.copy()
        if len(self.centers) > 0:
            for center in self.centers:
                frame_curr_with_edges = cv2.circle(
                    frame_curr_with_edges,
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
                frame_curr_with_edges = cv2.line(
                    frame_curr_with_edges,
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
        # endregion

        packed_frames = {
            "Process": "Markers",
            "frame_curr_with_flow_vectors": frame_curr_with_flow_vectors,
            "frame_curr_with_tris": frame_curr_with_tris,
            "frame_curr_with_edges": frame_curr_with_edges,
        }
        output_queue.put(packed_frames)

        if self.is_timing:
            print("Markers: Visualize Time: ", time.time() - tic)

    def update(self, frame_queue, result_queue, visualize_queue, is_init=False):
        """update marker tracking by calling all the functions."""
        frame = frame_queue.get()

        self.track(frame)
        self.segment(is_init=True)

        while True:
            print("Markers: frame_queue size: ", frame_queue.qsize())
            frame = frame_queue.get()

            self.track(frame)
            self.segment(is_init=False)
            self.edge()
            self.force(frame)

            if is_init:
                self.keypoints_queue.clear()
                self.keypoints_mask_queue.clear()
                self.tris_queue.clear()
                self.area_queue.clear()
                self.area_diff_queue.clear()
            elif self.is_visualize:
                self.visualize(visualize_queue)

    def run(self, frame_queue, result_queue, visualize_queue):
        """Starts the marker tracking process."""
        self.processes = []

        # if self.is_ros:
        #     self.processes.append(mp.Process(target=self.ros_listener))
        #     self.processes.append(mp.Process(target=self.ros_publisher))
        self.processes.append(mp.Process(target=self.update, args=(frame_queue, result_queue, visualize_queue)))

        [p.start() for p in self.processes]

    def stop(self):
        [p.terminate() for p in self.processes]


def main():
    gsmini = Device()
    markers = Markers()

    frame_queue = mp.Queue()
    result_queue = mp.Queue()
    visualize_queue = mp.Queue()

    gsmini.run(frame_queue)
    markers.run(frame_queue, result_queue, visualize_queue)

    # Report and Visualize
    try:
        while True:
            # packed_results = result_queue.get()
            packed_frames = visualize_queue.get()
            if packed_frames["Process"] == "Markers":
                cv2.imshow("Markers | Flow Vectors", packed_frames["frame_curr_with_flow_vectors"])
                cv2.imshow("Markers | Triangles", packed_frames["frame_curr_with_tris"])
                cv2.imshow("Markers | Edges", packed_frames["frame_curr_with_edges"])

            getKey = cv2.waitKey(1)
            if getKey & 0xFF == ord("q"):
                break
            if getKey & 0xFF == ord("r"):
                markers.is_init = True
    finally:
        gsmini.stop()
        markers.stop()


if __name__ == "__main__":
    main()
