import cv2
import time
import numpy as np
from global_params import *
import matplotlib.pyplot as plt
from track_deform import dotSegment
from connect_digit import connectDigit
from genetic_calc import calcMatrixM
from detect_blob import setDetectionParams, dotDetection
from track_markers import dotRegistration, dotMatching


def setVideoEncoder(scale=2):
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    videoOut = cv2.VideoWriter()
    timestr = time.strftime("%Y%m%d-%H%M%S")
    if ifVGA:
        videoOut.open(
            "output/markerDetect-{}.mp4".format(timestr),
            fourcc,
            30,
            (480 * scale, 640 * scale),
        )
    else:
        videoOut.open(
            "output/markerDetect-{}.mp4".format(timestr),
            fourcc,
            30,
            (240 * scale, 320 * scale),
        )
    return videoOut


def main():
    digit = connectDigit()
    for _ in range(30):  # Preheat the digit
        Frm0 = digit.get_frame()

    videoOut = setVideoEncoder()
    print(
        "Press ESC to quit, O to capture Original, C to capture Difference, D to show Difference."
    )

    blobDetector = cv2.SimpleBlobDetector_create(setDetectionParams())

    while True:
        Frm = digit.get_frame()

        ## Dot detection
        keypoints, Frm_with_keypoints = dotDetection(blobDetector, Frm)

        cv2.imshow("Preview", Frm_with_keypoints)
        cv2.moveWindow("Preview", 100, 100)

        getKey = cv2.waitKey(1)
        if getKey == 27 or getKey == ord("q"):  # ESC or q
            break
        if len(keypoints) == 0:  # No dot detected cause error
            continue
        elif getKey == ord("o"):  # Get original frame
            Frm_a = Frm
            keypoints_a = keypoints
            Frm_a_with_keypoints = Frm_with_keypoints
            print("Original Frame Got.")
            cv2.imshow("Original", Frm_a_with_keypoints)
            cv2.moveWindow("Original", 100, 550)
        elif getKey == ord("c"):  # Capture difference
            digit.show_view(digit.get_frame())
            cv2.waitKey(0)
            break
        elif getKey == ord("d"):  # Show difference
            print("Press s to save Data to file.")
            while True:
                Frm = digit.get_frame()
                Frm_b = Frm
                keypoints_b, Frm_b_with_keypoints = dotDetection(blobDetector, Frm)

                ## Temporary implementation
                if len(keypoints_a) != len(keypoints_b):
                    continue

                X, Y, TY, G, W, P = dotRegistration(keypoints_a, keypoints_b)
                Frm_dot_movement, dotPair = dotMatching(X, Y, TY, P, Frm_a, Frm_b)

                tri, Frm_dot_segment = dotSegment(Y, Frm_dot_movement, 2, (0, 255, 255))
                tri, Frm_dot_segment = dotSegment(X, Frm_dot_segment, 2, (255, 255, 0))

                videoOut.write(Frm_dot_segment)

                cv2.destroyWindow("Preview")
                cv2.moveWindow("Original", 100, 100)
                cv2.imshow("Dot Movement", Frm_dot_movement)
                cv2.moveWindow("Dot Movement", 490, 100)
                cv2.imshow("Dot Segment", Frm_dot_segment)
                cv2.moveWindow("Dot Segment", 490, 550)
                cv2.imshow("Current", Frm_b_with_keypoints)
                cv2.moveWindow("Current", 100, 550)
                getKey = cv2.waitKey(1)
                if getKey == ord("s"):
                    np.savetxt("output/saved_P.out", P, delimiter=",")
                    np.savetxt("output/saved_X.out", X, delimiter=" ")
                    np.savetxt("output/saved_Y.out", Y, delimiter=" ")
                    print("Data saved to file.")
                if getKey == 27 or getKey == ord("q"):  # ESC or q
                    break
            break

    ## Turn off the digit
    videoOut.release()
    print("Video saved to output/")
    digit.disconnect()


if __name__ == "__main__":
    main()
