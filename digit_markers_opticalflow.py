import cv2
import time
import numpy as np
from connect_gsmini import connectGSmini
from detect_blob import setDetectionParams, dotDetection


class Markers_OF:
    frame_init, frame_prev, frame_curr = None, None, None
    keypoints_init, keypoints_prev, keypoints_curr = None, None, None

    def __init__(self):
        self.blobDetector = cv2.SimpleBlobDetector_create(setDetectionParams())
        self.feature_params = dict(
            maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7
        )
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        self.isInitFrame = True
        self.isTiming = False
        return

    def __track_fallback(self, frame):
        # TODO: Track using old method

        return

    def __update_motion(self):

        return

    def detect(self, frame):
        if self.isTiming:
            tic = time.time()

        # region Get Keypoints
        self.frame_curr = frame
        self.keypoints_curr, self.frame_curr_with_keypoints = dotDetection(
            self.blobDetector, frame
        )
        self.keypoints_curr = np.array(
            [[[kp.pt[0], kp.pt[1]]] for kp in self.keypoints_curr], dtype=np.float32
        )
        # endregion

        # region Optical Flow
        if (
            not self.isInitFrame
        ):  # After Init Frame: Calculate Optical Flow between previous and current frame
            self.p1_curr, self.st_curr, self.err_curr = cv2.calcOpticalFlowPyrLK(
                self.frame_prev,
                self.frame_curr,
                self.keypoints_prev,
                self.keypoints_curr,
                **self.lk_params
            )
        # endregion

        # TODO: Displacement, Speed, Accleration Calculation Here

        if self.isTiming:
            print("Markers: Update Time: ", time.time() - tic)
        return

    def track(self):
        """Update the paring of markers from the init frame through successive frames."""
        if self.isTiming:
            tic = time.time()

        # TODO: Implement Tracking
        raise NotImplementedError

        if self.isTiming:
            print("Markers: Track Time: ", time.time() - tic)
        return

    def visualize(self):
        """Visualize the keypoints movement on the current frame."""
        if self.isTiming:
            tic = time.time()

        # region Visualize: Keypoints Movement
        if not self.isInitFrame:
            for i, (new, old) in enumerate(zip(self.p1_curr, self.keypoints_prev)):
                a, b = new.ravel()
                c, d = old.ravel()
                a = int(a)
                b = int(b)
                c = int(c)
                d = int(d)
                cv2.arrowedLine(
                    self.frame_curr_with_keypoints,
                    (a - (a - c) * 2, b - (b - d) * 2),
                    (c, d),
                    (124, 67, 237),  # KizunaAI Color
                    5,
                    cv2.LINE_AA,
                    tipLength=0.2,
                )
                # cv2.circle(frm, (a, b), 5, (0, 0, 255), -1)
            cv2.imshow("Optical Flow", self.frame_curr_with_keypoints)
        # endregion

        if self.isTiming:
            print("Markers: Visualize Time: ", time.time() - tic)
        return

    def update(self):

        self.frame_prev = self.frame_curr
        self.keypoints_prev = self.keypoints_curr

        # TODO: Check if this is correct, maybe needs to copy the instance
        if not self.isInitFrame:
            self.p1_prev, self.st_prev, self.err_prev = (
                self.p1_curr,
                self.st_curr,
                self.err_curr,
            )

        if self.isInitFrame:
            self.isInitFrame = False


def main():
    gsmini = connectGSmini()
    blobDetector = cv2.SimpleBlobDetector_create(setDetectionParams())

    frame_init = gsmini.get_image((320, 240))
    markers = Markers_OF()
    try:
        while True:
            tic = time.time()

            # region Main Loop
            frame = gsmini.get_image((320, 240))

            markers.detect(frame)
            # markers.track()
            markers.visualize()
            markers.update()
            # endregion

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            toc = time.time()
            print("FPS: ", 1 / (toc - tic))
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    return


if __name__ == "__main__":
    main()
