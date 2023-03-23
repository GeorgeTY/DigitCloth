import sys
import cv2
import time
import numpy as np
from collections import deque
from connect_gsmini import connectGSmini
from detect_blob import setDetectionParams, dotDetection


class Markers:  # Optical Flow
    sample_length = 128
    dots_count = 284

    frame_queue = deque(maxlen=sample_length)
    keypoints_queue = deque(maxlen=sample_length)
    keypoints_mask_queue = deque(maxlen=sample_length)

    def __init__(self):
        self.blob_detector = cv2.SimpleBlobDetector_create(setDetectionParams())
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=1, blockSize=2)  # 100, 0.3, 7, 7
        self.lk_params = dict(
            winSize=(20, 20),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1, 0.03),  # 10, 0.03
        )
        self.is_timing = False
        self.is_visualize = True

    def get_keypoints(self):
        return self.keypoints_queue[-1].squeeze()  # Remove unwanted 1 dimension

    def get_frame(self):
        return self.frame_queue[-1]

    def init(self, frame):
        self.frame_queue.clear()
        self.keypoints_queue.clear()
        self.keypoints_mask_queue.clear()
        self.track(frame, is_init=True)

    def track(self, frame, is_init=False):
        """Track the markers in the current frame."""
        tic = time.time()

        if len(self.frame_queue) == self.sample_length:
            self.frame_queue.popleft()
            self.keypoints_queue.popleft()
            self.keypoints_mask_queue.popleft()
        self.frame_queue.append(frame)

        keypoints, self.frame_curr_with_keypoints = dotDetection(self.blob_detector, frame)
        if is_init:
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
            cv2.moveWindow("Optical Flow", 200, 100)
        # endregion

        if self.is_timing:
            print("Markers: Visualize Time: ", time.time() - tic)

    def update(self, frame, is_init=False):
        """update marker tracking by calling all the functions."""

        self.track(frame)
        if is_init:
            self.frame_queue.clear()
            self.keypoints_queue.clear()
            self.keypoints_mask_queue.clear()
        else:
            if self.is_visualize:
                self.visualize()


def main():
    gsmini = connectGSmini()
    markers = Markers()

    for _ in range(5):
        _ = gsmini.get_image((384, 288))

    try:
        while True:
            tic = time.time()

            frame = gsmini.get_image((384, 288))
            markers.update(frame)

            getKey = cv2.waitKey(1)
            if getKey & 0xFF == ord("q"):
                break
            elif getKey & 0xFF == ord("r"):
                markers.init(frame)
            toc = time.time()
            # print("FPS: ", 1 / (toc - tic))
            sys.stdout.write(f"FPS: {1 / (toc - tic):.2f}\r")
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        cv2.destroyAllWindows()
        gsmini.stop_video()


if __name__ == "__main__":
    main()