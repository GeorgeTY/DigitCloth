import os
import sys
import cv2
import time
import rospy
import platform
import numpy as np
from global_params import *
import matplotlib.pyplot as plt
from detect_markers import Markers_OF
from ros_comms import ros_talker
from record_digit import setVideoEncoder
from connect_gsmini import connectGSmini


def main():
    gsmini = connectGSmini()
    markers = Markers_OF()

    for _ in range(5):
        _ = gsmini.get_image((384, 288))

    try:
        while True:
            tic = time.time()

            frame = gsmini.get_image((384, 288))
            markers.run(frame)

            getKey = cv2.waitKey(1)
            if getKey & 0xFF == ord("q"):
                break
            elif getKey & 0xFF == ord("r"):
                markers.reset()

            toc = time.time()
            sys.stdout.write(f"FPS: {1 / (toc - tic):.2f}\r")
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        cv2.destroyAllWindows()
        gsmini.stop_video()


if __name__ == "__main__":
    main()
