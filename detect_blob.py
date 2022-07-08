import cv2
import numpy as np
from global_params import *


def setDetectionParams():
    blobParams = cv2.SimpleBlobDetector_Params()

    blobParams.minThreshold = 0
    blobParams.maxThreshold = 255

    if ifVGA:
        blobParams.filterByArea = True
        blobParams.minArea = 50
        blobParams.maxArea = 500
    else:
        blobParams.filterByArea = True
        blobParams.minArea = 20
        blobParams.maxArea = 100

    blobParams.filterByCircularity = True
    blobParams.minCircularity = 0.7

    # blobParams.filterByConvexity = True
    # blobParams.minConvexity = 0.8

    # blobParams.filterByInertia = True
    # blobParams.minInertiaRatio = 0.01

    return blobParams


def dotDetection(blobDetector, Frm, circleColor=(0, 0, 255)):
    keypoints = blobDetector.detect(Frm)
    Frm_with_keypoints = cv2.drawKeypoints(
        Frm,
        keypoints,
        np.array([]),
        (50, 50, 150),
        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )
    for keypoint in keypoints:
        cv2.circle(
            Frm_with_keypoints,
            (int(keypoint.pt[0]), int(keypoint.pt[1])),
            1,
            circleColor,
            -1,
            cv2.LINE_AA,
        )
    Frm_with_keypoints = cv2.putText(
        Frm_with_keypoints,
        "{}".format(len(keypoints)),
        (5, 315),
        cv2.FONT_HERSHEY_COMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return keypoints, Frm_with_keypoints


def main():
    pass


if __name__ == "__main__":
    main()
