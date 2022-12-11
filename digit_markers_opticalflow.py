import cv2
import time
import numpy as np
from connect_gsmini import connectGSmini
from detect_blob import setDetectionParams, dotDetection


class Markers_OF:
    frame_init, frame_prev, frame_curr = None, None, None
    keypoints_init, keypoints_prev, keypoints_curr = None, None, None
    sample_length = 128

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
        self.isVisualize = True
        return

    def __track_fallback(self, frame):
        # TODO: Track using old method

        return

    def track(self, frame):
        """Track the markers in the current frame."""
        if self.isTiming:
            tic = time.time()

        # region Get Keypoints
        self.frame_curr = frame
        # if self.isInitFrame:  # Init Frame: Get Keypoints by blob detection
        if True:
            self.keypoints_curr, self.frame_curr_with_keypoints = dotDetection(
                self.blobDetector, frame
            )
            self.keypoints_curr = np.array(
                [[[kp.pt[0], kp.pt[1]]] for kp in self.keypoints_curr], dtype=np.float32
            )
        else:
            self.keypoints_curr = self.keypoints_prev
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

        if self.isTiming:
            print("Markers: Update Time: ", time.time() - tic)
        return

    def pairing(self):
        """Update the paring of markers from the init frame through successive frames."""
        if self.isTiming:
            tic = time.time()

        # TODO: Implement Pairing

        if self.isTiming:
            print("Markers: Track Time: ", time.time() - tic)
        return

    def calc(self):
        """Calculate the displacement, speed, and acceleration of the markers."""

        return

    def visualize(self):
        """Visualize the keypoints movement on the current frame."""
        if self.isTiming:
            tic = time.time()

        # region Visualize: Keypoints Movement
        if not self.isInitFrame:
            self.frame_curr_with_flow_vectors = cv2.addWeighted(
                self.frame_prev, 0.5, self.frame_curr_with_keypoints, 0.5, 0
            )

            for i, (new, old) in enumerate(zip(self.p1_curr, self.keypoints_prev)):
                a, b = new.ravel()
                c, d = old.ravel()
                a = int(a)
                b = int(b)
                c = int(c)
                d = int(d)
                cv2.arrowedLine(
                    self.frame_curr_with_flow_vectors,
                    (a - (a - c) * 2, b - (b - d) * 2),
                    (c, d),
                    (124, 67, 237),  # KizunaAI Color
                    2,
                    cv2.LINE_AA,
                    tipLength=0.25,
                )

            cv2.putText(
                self.frame_curr_with_flow_vectors,
                str(len(self.p1_curr)),
                (0, 15),
                cv2.FONT_HERSHEY_COMPLEX,
                0.5,
                (255, 255, 255),
            )
            cv2.imshow("Optical Flow", self.frame_curr_with_flow_vectors)
        # endregion

        if self.isTiming:
            print("Markers: Visualize Time: ", time.time() - tic)
        return

    def update(self):
        """Update the previous frame and keypoints."""

        self.frame_prev = self.frame_curr
        self.keypoints_prev = self.keypoints_curr

        if not self.isInitFrame:
            self.p1_prev, self.st_prev, self.err_prev = (
                self.p1_curr,
                self.st_curr,
                self.err_curr,
            )

        if self.isInitFrame:
            self.isInitFrame = False

    def run(self, frame):
        """Run marker tracking by calling all the functions."""

        self.track(frame)
        self.pairing()
        self.calc()
        if self.isVisualize:
            self.visualize()
        self.update()

        return


def main():
    gsmini = connectGSmini()
    gsmini.imgw = 288  # Integer Scaling
    gsmini.imgh = 384
    blobDetector = cv2.SimpleBlobDetector_create(setDetectionParams())

    frame_init = gsmini.get_image((384, 288))
    markers = Markers_OF()
    try:
        while True:
            tic = time.time()

            frame = gsmini.get_image((384, 288))
            markers.run(frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            toc = time.time()
            print("FPS: ", 1 / (toc - tic))
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        cv2.destroyAllWindows()
        gsmini.stop_video()
    return


if __name__ == "__main__":
    main()
