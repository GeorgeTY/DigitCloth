import cv2
import numpy as np
from global_params import *


def setDetectionParams():
    blobParams = cv2.SimpleBlobDetector_Params()

    blobParams.minThreshold = minThreshold
    blobParams.maxThreshold = maxThreshold

    if ifVGA:
        blobParams.filterByArea = True
        blobParams.minArea = minArea_VGA
        blobParams.maxArea = maxArea_VGA
    else:
        blobParams.filterByArea = True
        blobParams.minArea = minArea
        blobParams.maxArea = maxArea

    blobParams.filterByCircularity = True
    blobParams.minCircularity = minCircularity

    blobParams.filterByConvexity = False
    blobParams.minConvexity = minConvexity

    blobParams.filterByInertia = False
    blobParams.minInertiaRatio = minInertiaRatio

    return blobParams


def dotDetection(blobDetector, frm, circleColor=(0, 0, 255)):
    keypoints = blobDetector.detect(frm)
    frm_with_keypoints = cv2.drawKeypoints(
        frm,
        keypoints,
        np.array([]),
        (50, 50, 150),
        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )
    for keypoint in keypoints:
        cv2.circle(
            frm_with_keypoints,
            (int(keypoint.pt[0]), int(keypoint.pt[1])),
            1,
            circleColor,
            -1,
            cv2.LINE_AA,
        )
    frm_with_keypoints = cv2.putText(
        frm_with_keypoints,
        "{}".format(len(keypoints)),
        (5, 315),
        cv2.FONT_HERSHEY_COMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return keypoints, frm_with_keypoints


def main():
    pass


if __name__ == "__main__":
    main()
