from connect_gsmini import connectGSmini
from detect_blob import setDetectionParams, dotDetection
import cv2
import numpy as np
import time


def main():
    gsmini = connectGSmini()
    blobDetector = cv2.SimpleBlobDetector_create(setDetectionParams())

    frm0 = gsmini.get_image((320, 240))
    # frm0 = cv2.imread("pics/digit_movement/frm0.png")
    keypoints0, frm_with_keypoints0 = dotDetection(blobDetector, frm0)
    keypoints0lk = [[[kp.pt[0], kp.pt[1]]] for kp in keypoints0]
    keypoints0lk = np.array(keypoints0lk, dtype=np.float32)

    feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

    lk_params = dict(
        winSize=(15, 15),
        maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
    )
    frm0_gray = cv2.cvtColor(frm0, cv2.COLOR_BGR2GRAY)
    keypointsgood = cv2.goodFeaturesToTrack(frm0_gray, mask=None, **feature_params)
    # print(keypointsgood)
    # keypoints0 = [
    #     cv2.KeyPoint(x=kp.pt[0], y=kp.pt[1], size=kp.size) for kp in keypoints0
    # ]
    try:

        while True:
            tic = time.time()
            frm = gsmini.get_image((320, 240))
            # frm = cv2.rotate(frm, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # frm = cv2.imread("pics/digit_movement/frm.png")
            # detect blob
            keypoints, frm_with_keypoints = dotDetection(blobDetector, frm)
            cv2.imshow("frm", frm_with_keypoints)

            # cv2.simpleblobdetector keypoints to cv2.calcOpticalFlowPyrLK conversion
            keypointslk = [[[kp.pt[0], kp.pt[1]]] for kp in keypoints]
            keypointslk = np.array(keypointslk, dtype=np.float32)
            # print("keypoints", keypointslk)

            p1, st, err = cv2.calcOpticalFlowPyrLK(
                frm0, frm, keypoints0lk, keypointslk, **lk_params
            )
            ## optical flow visualization
            for i, (new, old) in enumerate(zip(p1, keypoints0lk)):
                a, b = new.ravel()
                c, d = old.ravel()
                a = int(a)
                b = int(b)
                c = int(c)
                d = int(d)
                cv2.line(
                    frm, (a + (a - c) * 2, b + (b - d) * 2), (c, d), (0, 255, 0), 2
                )
                # cv2.circle(frm, (a, b), 5, (0, 0, 255), -1)
            cv2.imshow("frm", frm)
            keypoints0lk = keypointslk
            frm0 = frm

            toc = time.time()
            print("FPS: ", 1 / (toc - tic))

            getKey = cv2.waitKey(1)
            if getKey == 27 or getKey == ord("q"):  # ESC or q
                break

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        gsmini.stop_video()
        cv2.destroyAllWindows()
    return


if __name__ == "__main__":
    main()
