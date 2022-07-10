import os
import cv2
import time
import numpy as np
from global_params import *
import matplotlib.pyplot as plt
from connect_digit import connectDigit
from genetic_calc import calcMatrixM
from record_digit import setVideoEncoder
from detect_blob import setDetectionParams, dotDetection
from track_markers import dotRegistration, dotMatching
from track_deform import dotSegment, drawSegment, getAreaDiff, pltDeform, drawArea


def main():
    if ifRec:
        cap = cv2.VideoCapture("./output/recordDigit.mp4")
        print("Reading video...")
    else:
        digit = connectDigit(intensity)
        for _ in range(15):  # Preheat the digit
            Frm0 = digit.get_frame()

    timestr = time.strftime("%Y%m%d-%H%M%S")
    fileName = "output/markerDetect-{}.mp4".format(timestr)
    videoOut, videotempName, videofileName = setVideoEncoder(fileName)
    videoSave = False
    print(
        "Press ESC to quit, O to capture Original, C to capture Difference, D to show Difference."
    )

    blobDetector = cv2.SimpleBlobDetector_create(setDetectionParams())

    while True:
        if ifRec:
            Frm = cap.read()
        else:
            Frm = digit.get_frame()

        ## Dot detection
        keypoints, Frm_with_keypoints = dotDetection(blobDetector, Frm)

        cv2.imshow("Preview", Frm_with_keypoints)
        cv2.moveWindow("Preview", 2020, 100)

        if ifRec:
            getKey = cv2.waitKey(0)
        else:
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

            X = np.array([keypoint.pt for keypoint in keypoints_a])
            tri_a, area_a, Frm_a_dot_segment = dotSegment(
                X,
                Frm_a_with_keypoints,
                1,
                (0, 255, 255),
            )

            cv2.imshow("Original", Frm_a_dot_segment)
            cv2.moveWindow("Original", 2020, 550)
        elif getKey == ord("c"):  # Capture difference
            if ifRec:
                continue
            digit.show_view(digit.get_frame())
            cv2.waitKey(0)
            break
        elif getKey == ord("d"):  # Show difference
            print("Press s to save Data to file.")
            while True:
                tic = time.time()

                if ifRec:
                    Frm = cap.read()
                else:
                    Frm = digit.get_frame()

                Frm_b = Frm
                keypoints_b, Frm_b_with_keypoints = dotDetection(blobDetector, Frm)
                if len(keypoints_b) == 0:  # No dot detected cause error
                    continue

                ##### Temporary implementation #####
                if len(keypoints_a) != len(keypoints_b):
                    continue
                ####################################

                X, Y, TY, G, W, P = dotRegistration(keypoints_a, keypoints_b)
                Frm_dot_movement, dotPair = dotMatching(X, Y, TY, P, Frm_a, Frm_b)

                tri_b, area_b, Frm_b_dot_segment = drawSegment(
                    Y,
                    tri_a,
                    np.transpose(dotPair),
                    Frm_dot_movement,
                )
                area_diff = getAreaDiff(area_a, area_b)
                Frm_b_dot_segment = drawArea(
                    Y, tri_a, np.transpose(dotPair), area_diff, Frm_b_dot_segment
                )

                videoOut.write(Frm_b_dot_segment)

                pltDeform(np.matmul(np.transpose(dotPair), Y), tri_a, area_b, area_diff)
                # plt.get_current_fig_manager().window.setGeometry = (200, 550, 480, 640)
                plt.ion()
                plt.pause(1e-12)

                cv2.destroyWindow("Preview")
                cv2.moveWindow("Original", 2020, 100)
                cv2.imshow("Dot Movement", Frm_dot_movement)
                cv2.moveWindow("Dot Movement", 2410, 100)
                cv2.imshow("Dot Segment", Frm_b_dot_segment)
                cv2.moveWindow("Dot Segment", 2920, 100)
                cv2.imshow("Current", Frm_b_with_keypoints)
                cv2.moveWindow("Current", 2020, 550)
                getKey = cv2.waitKey(1)
                if getKey == ord("s"):
                    np.savetxt("output/saved_P.out", P, delimiter=",")
                    np.savetxt("output/saved_X.out", X, delimiter=" ")
                    np.savetxt("output/saved_Y.out", Y, delimiter=" ")
                    print("Data saved to file.")
                    videoSave = True
                    print("Video saved to output/")
                if getKey == 27 or getKey == ord("q"):  # ESC or q
                    break

                toc = time.time()
                print("FPS: %.2f" % (1 / (toc - tic)))
            break

    ## Turn off the digit
    videoOut.release()
    if videoSave:
        os.rename(videotempName, videofileName)
    else:
        os.remove(videotempName)
    plt.ioff()
    if ifRec:
        cap.release()
    else:
        digit.disconnect()


if __name__ == "__main__":
    main()
