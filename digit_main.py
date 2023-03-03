import os
import sys
import cv2
import time
import rospy
import platform
import numpy as np
from global_params import *
import matplotlib.pyplot as plt
from detect_markers import Markers
from detect_deform import DelaunayTri
from detect_edges import Edges
from ros_comms import ros_talker
from record_digit import setVideoEncoder
from connect_gsmini import connectGSmini


def main():
    gsmini = connectGSmini()
    markers = Markers()
    delaunaytris = DelaunayTri()
    edges = Edges()

    for _ in range(5):
        frame = gsmini.get_image((384, 288))  # ROI doesn't do anything yet

    try:
        markers.init(frame)
        delaunaytris.init(markers.get_keypoints())
        while True:
            tic = time.time()

            frame = gsmini.get_image((384, 288))
            markers.update(frame)
            delaunaytris.update(markers.get_frame(), markers.get_keypoints())
            edges.update(
                markers.get_frame(),
                delaunaytris.get_visualized_frame(),
                markers.get_keypoints(),
                delaunaytris.tris,
                delaunaytris.area_curr,
                delaunaytris.area_diff,
            )

            getKey = cv2.waitKey(1)
            if getKey & 0xFF == ord("q"):
                break
            elif getKey & 0xFF == ord("r") or getKey & 0xFF == ord("i"):
                markers.init(frame)
                delaunaytris.init(markers.get_keypoints())

            toc = time.time()
            sys.stdout.write(f"FPS: {1 / (toc - tic):.2f}\r")
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        cv2.destroyAllWindows()
        gsmini.stop_video()


if __name__ == "__main__":
    main()
