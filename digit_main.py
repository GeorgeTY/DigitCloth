import os
import cv2
import time
import platform
import numpy as np
from global_params import *
import matplotlib.pyplot as plt

if not (platform.system() == "Windows" or platform.system() == "Darwin"):
    from connect_digit import connectDigit

from fix_blob import fixMissingBlob
from genetic_calc import calcMatrixM
from record_digit import setVideoEncoder
from detect_edge import edgeDetection, edgeVisualize
from detect_blob import setDetectionParams, dotDetection
from track_markers import dotRegistration, dotMatching
from track_deform import dotSegment, drawSegment, getAreaDiff, pltDeform, drawArea


def main():
    if ifRec:
        cap = cv2.VideoCapture("./output/recordDigit.mp4")
        print("Reading video...")
    if platform.system() == "Windows" or platform.system() == "Darwin":
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        print("Reading camera...")
    else:
        digit = connectDigit(intensity)
        for _ in range(15):  # Preheat the digit
            frm0 = digit.get_frame()

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
            frm = cap.read()
        if platform.system() == "Windows" or platform.system() == "Darwin":
            ret, frm = cap.read()
            frm = cv2.rotate(frm, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            frm = digit.get_frame().copy()

        ## Dot detection
        keypoints, frm_with_keypoints = dotDetection(blobDetector, frm)

        cv2.imshow("Preview", frm_with_keypoints)
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
            frm_a = frm
            keypoints_a = keypoints
            frm_a_with_keypoints = frm_with_keypoints
            print("Original Frame Got.")

            X = np.array([keypoint.pt for keypoint in keypoints_a])
            tri_a, area_a, frm_a_dot_segment = dotSegment(
                X,
                frm_a_with_keypoints,
                1,
                (0, 255, 255),
            )

            cv2.imshow("Original", frm_a_dot_segment)
            cv2.moveWindow("Original", 2020, 480)
        elif getKey == ord("c"):  # Capture difference
            if ifRec:
                continue
            digit.show_view(digit.get_frame().copy())
            cv2.waitKey(0)
            break
        elif getKey == ord("d"):  # Show difference
            print("Press s to save Data to file.")
            cv2.destroyWindow("Preview")
            while True:
                tic = time.time()

                if ifRec:
                    frm = cap.read()
                if platform.system() == "Windows" or platform.system() == "Darwin":
                    ret, frm = cap.read()
                    frm = cv2.rotate(frm, cv2.ROTATE_90_COUNTERCLOCKWISE)
                else:
                    frm = digit.get_frame().copy()

                frm_b = frm
                keypoints_b, frm_b_with_keypoints = dotDetection(blobDetector, frm)
                if len(keypoints_b) == 0:  # No dot detected cause error
                    cv2.imshow("Current", frm_b_with_keypoints)
                    cv2.moveWindow("Current", 2020, 480)
                    cv2.waitKey(1)
                    continue

                ##### Temporary implementation #####
                if len(keypoints_a) != len(keypoints_b):
                    cv2.imshow("Current", frm_b_with_keypoints)
                    cv2.moveWindow("Current", 2020, 480)
                    getKet = cv2.waitKey(1)
                    if getKey == 27 or getKey == ord("q"):
                        break
                    continue

                    fixMissingBlob()
                ####################################

                X, Y, TY, G, W, P = dotRegistration(keypoints_a, keypoints_b)
                frm_dot_movement, dotPair = dotMatching(X, Y, TY, P, frm_a, frm_b)

                tri_b, area_b, frm_b_dot_segment = drawSegment(
                    Y,
                    tri_a,
                    np.transpose(dotPair),
                    frm_dot_movement,
                )
                print("avg area:", np.average(area_b))
                area_diff = getAreaDiff(area_a, area_b)
                frm_b_dot_segment = drawArea(
                    Y, tri_a, np.transpose(dotPair), area_diff, frm_b_dot_segment
                )

                result, frm_b_edge_detected = edgeDetection(
                    tri_a,
                    Y,
                    np.transpose(dotPair),
                    area_b,
                    area_diff,
                    frm_b_dot_segment,
                    method=ed_method,
                    deg=1,
                    scale=scale,
                )
                if frm_b_edge_detected is not None and result is not None:
                    frm_b_edge_detected = edgeVisualize(
                        frm_b_edge_detected, result, method=ed_method
                    )

                # videoOut.write(frm_b_dot_segment)

                if frm_b_edge_detected is not None:
                    videoOut.write(frm_b_edge_detected)
                    cv2.imshow("Edge Detection", frm_b_edge_detected)
                    cv2.moveWindow("Edge Detection", 3280, 100)
                else:
                    videoOut.write(frm_b_dot_segment)

                pltDeform(np.matmul(np.transpose(dotPair), Y), tri_a, area_b, area_diff)
                # plt.get_current_fig_manager().window.setGeometry = (200, 550, 480, 640)
                plt.ion()
                plt.pause(1e-12)

                cv2.moveWindow("Original", 2020, 100)
                cv2.imshow("Dot Movement", frm_dot_movement)
                cv2.moveWindow("Dot Movement", 2280, 100)
                cv2.imshow("Dot Segment", frm_b_dot_segment)
                cv2.moveWindow("Dot Segment", 2780, 100)
                cv2.imshow("Current", frm_b_with_keypoints)
                cv2.moveWindow("Current", 2020, 480)
                getKey = cv2.waitKey(1)
                if getKey == ord("s"):
                    cv2.imwrite("output/saved_frm.png", frm_b_dot_segment)
                    np.savetxt("output/saved_P.out", P, delimiter=",")
                    np.savetxt("output/saved_X.out", X, delimiter=" ")
                    np.savetxt("output/saved_Y.out", Y, delimiter=" ")
                    # np.savetxt("output/saved_tri.out", tri_a, delimiter=" ")
                    np.savetxt("output/saved_area.out", area_b, delimiter=" ")
                    np.savetxt("output/saved_area_diff.out", area_diff, delimiter=" ")
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
    if ifRec or platform.system() == "Windows" or platform.system() == "Darwin":
        cap.release()
    else:
        digit.disconnect()


if __name__ == "__main__":
    main()
