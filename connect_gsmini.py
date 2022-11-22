from gelsight import gsdevice
import cv2
import numpy as np
from global_params import *
from detect_blob import setDetectionParams, dotDetection


def connectGSmini():
    cam_id = 0
    dev = gsdevice.Camera(gsdevice.Finger.MINI, cam_id)
    dev.connect()

    return dev


def main():
    gsmini = connectGSmini()

    blobDetector = cv2.SimpleBlobDetector_create(setDetectionParams())

    try:
        while True:
            frame = gsmini.get_image((320, 240))

            # Detect the dot
            keypoints, frm_with_keypoints = dotDetection(blobDetector, frame)

            # Show the image
            cv2.imshow("gsmini", frm_with_keypoints)
            getKey = cv2.waitKey(1)
            if getKey == 27 or getKey == ord("q"):  # ESC or q
                break
    except KeyboardInterrupt:
        gsmini.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
