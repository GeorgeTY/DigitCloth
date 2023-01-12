import sys
import cv2
import time
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from connect_gsmini import connectGSmini
from detect_blob import setDetectionParams, dotDetection


class Markers_OF:  # Optical Flow
    sample_length = 128
    dots_count = 284

    frame_queue = deque(maxlen=sample_length)
    keypoints_queue = deque(maxlen=sample_length)
    keypoints_mask_queue = deque(maxlen=sample_length)
    timestamp_queue = deque(maxlen=sample_length)
    acceleration_queue = deque(maxlen=sample_length)
    velocity_queue = deque(maxlen=sample_length)

    def __init__(self):
        self.blob_detector = cv2.SimpleBlobDetector_create(setDetectionParams())
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7) # 100, 0.3, 7, 7
        self.lk_params = dict(
            winSize=(20, 20),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        self.is_first_frame = True
        self.is_timing = False
        self.is_visualize = True

    def __track_fallback(self, frame):
        # TODO: Track using old method
        raise NotImplementedError

    def track(self, frame):
        """Track the markers in the current frame."""
        tic = time.time()
        self.timestamp_queue.append(tic)

        if len(self.frame_queue) == self.sample_length:
            self.frame_queue.popleft()
            self.keypoints_queue.popleft()
            self.keypoints_mask_queue.popleft()
            self.timestamp_queue.popleft()
        self.frame_queue.append(frame)

        keypoints, self.frame_curr_with_keypoints = dotDetection(self.blob_detector, frame)
        if self.is_first_frame:
            # Capture the first frame and find the initial keypoints
            keypoint_count = len(keypoints)
            # (num_points, 1, 2)
            keypoints = np.array([[[p.pt[0], p.pt[1]]] for p in keypoints], dtype=np.float32)
            keypoints_mask = np.ones((keypoint_count,), dtype=np.int8)

            self.keypoints_queue.append(keypoints)
            self.keypoints_mask_queue.append(keypoints_mask)
        else:
            # Optical flow
            prev_frame, next_frame = self.frame_queue[-2], self.frame_queue[-1]
            prev_keypoints = self.keypoints_queue[-1]
            prev_keypoints_mask = self.keypoints_mask_queue[-1]

            available_keypoints_indices = np.nonzero(prev_keypoints_mask)[0]
            available_prev_keypoints = prev_keypoints[available_keypoints_indices]
            available_next_keypoints = np.zeros_like(available_prev_keypoints)

            available_next_keypoints, status, _ = cv2.calcOpticalFlowPyrLK(
                prevImg=prev_frame, nextImg=next_frame,
                prevPts=available_prev_keypoints, nextPts=available_next_keypoints,
                **self.lk_params
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

        if self.is_timing:
            print("Markers: Update Time: ", time.time() - tic)

    def calc(self, stride: int = 1):
        """Calculate the velocity, and acceleration of the markers."""

        if len(self.velocity_queue) == self.sample_length:
            self.velocity_queue.popleft()
        if len(self.acceleration_queue) == self.sample_length:
            self.acceleration_queue.popleft()

        if len(self.keypoints_queue) >= stride + 1:
            prev_keypoints, curr_keypoints = self.keypoints_queue[-1 - stride], self.keypoints_queue[-1]
            elapsed = self.timestamp_queue[-1] - self.timestamp_queue[-1 - stride]
            velocity = (curr_keypoints - prev_keypoints) / elapsed
            self.velocity_queue.append(velocity)

        if len(self.velocity_queue) >= stride + 2:
            prev_velocity, curr_velocity = self.velocity_queue[-1 - stride], self.velocity_queue[-1]
            elapsed = (self.timestamp_queue[-1] - self.timestamp_queue[-2 - stride]) / (stride + 1)
            acceleration = (curr_velocity - prev_velocity) / elapsed
            self.acceleration_queue.append(acceleration)

    def clustering(self):
        keypoints = self.keypoints_queue[-1].reshape(-1, 2)
        keypoints_mask = self.keypoints_mask_queue[-1]
        available_keypoints_indices = np.nonzero(keypoints_mask)[0]
        available_keypoints = keypoints[available_keypoints_indices]

        clustering = DBSCAN(eps=12.5, min_samples=4).fit(available_keypoints) # TODO: Tune parameters (important)
        unique_labels = set(clustering.labels_)
        core_samples_mask = np.zeros_like(clustering.labels_, dtype=bool)
        core_samples_mask[clustering.core_sample_indices_] = True

        if self.is_visualize and True:
            plt.cla()
            colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
            for k, col in zip(unique_labels, colors):
                if k == -1:
                    col = [0, 0, 0, 1]

                class_member_mask = (clustering.labels_ == k)

                xy = available_keypoints[class_member_mask & core_samples_mask]
                plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=14)

                xy = available_keypoints[class_member_mask & ~core_samples_mask]
                plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=6)
            plt.title('Estimated number of clusters: %d' % len(unique_labels))
            plt.draw()
            plt.pause(0.0001)


    def visualize(self):
        """Visualize the keypoints movement on the current frame."""
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
            cv2.imshow("Optical Flow", frame_curr_with_flow_vectors)
        # endregion

        stride = 2
        # region Visualize: Keypoints Velocity
        if len(self.velocity_queue) >= stride:
            frame_curr_with_velocity_keypoints = self.frame_queue[-1].copy()
            velocity_list = [self.velocity_queue[-i - 1] for i in range(stride)]
            velocity_list = np.mean(np.stack(velocity_list), axis=0)
            for i, (keypoint, velocity) in enumerate(zip(curr_keypoints, velocity_list)):
                if curr_keypoints_mask[i]:
                    a, b = keypoint.flatten()
                    c, d = velocity.flatten()
                    a, b,c,d = int(a), int(b), int(c), int(d)
                    cv2.arrowedLine(
                        frame_curr_with_velocity_keypoints,
                        (a, b),
                        (a + c, b + d),
                        (100,255,100),
                        2,
                        cv2.LINE_AA,
                        tipLength=0.25,
                    )
            cv2.imshow("Velocity", frame_curr_with_velocity_keypoints)
        # endregion

        # region Visualize: Keypoints Acceleration:
        if len(self.acceleration_queue) >= stride:
            frame_curr_with_acceleration_keypoints = self.frame_queue[-1].copy()
            acceleration_list = [self.acceleration_queue[-i - 1] for i in range(stride)]
            acceleration_list = np.mean(np.stack(acceleration_list), axis=0)
            for i, (keypoint, acceleration) in enumerate(zip(curr_keypoints, acceleration_list)):
                if curr_keypoints_mask[i]:
                    a, b = keypoint.flatten()
                    c, d = acceleration.flatten()
                    a, b,c,d = int(a), int(b),int(c),int(d)
                    cv2.arrowedLine(
                        frame_curr_with_acceleration_keypoints,
                        (a, b),
                        (a + c,b+d),
                        (100,100,255),
                        2,
                        cv2.LINE_AA,
                        tipLength=0.25,
                    )
            cv2.imshow("Acceleration", frame_curr_with_acceleration_keypoints)
        # endregion

        # region WindowMovement
        cv2.moveWindow("Optical Flow", 200, 100)
        cv2.moveWindow("Velocity", 600, 100)
        cv2.moveWindow("Acceleration", 1000, 100)
        # endregion

        if self.is_timing:
            print("Markers: Visualize Time: ", time.time() - tic)
   
    def run(self, frame):
        """Run marker tracking by calling all the functions."""

        self.track(frame)
        if self.is_first_frame:
            self.is_first_frame = False
        else:
            self.calc()
            self.clustering()
            if self.is_visualize:
                self.visualize()
            # raise RuntimeError("Stop")


def main():
    gsmini = connectGSmini()
    markers = Markers_OF()

    for _ in range(10):
        _ = gsmini.get_image((384, 288))

    try:
        while True:
            tic = time.time()

            frame = gsmini.get_image((384, 288))
            markers.run(frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            toc = time.time()
            # print("FPS: ", 1 / (toc - tic))
            sys.stdout.write(f"FPS: {1 / (toc - tic)}\r")
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        cv2.destroyAllWindows()
        gsmini.stop_video()


if __name__ == "__main__":
    main()
