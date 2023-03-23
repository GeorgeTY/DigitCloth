import cv2 as cv
import time
import sys
import numpy as np


# The video feed is read in as
# a VideoCapture object
# cap = cv.VideoCapture("./saved/video/clothMovement_0.webm")
cap = cv.VideoCapture(0)

for _ in range(10):
    _, _ = cap.read()

# ret = a boolean return value from
# getting the frame, first_frame = the
# first frame in the entire video sequence
ret, first_frame = cap.read()

# Converts frame to grayscale because we
# only need the luminance channel for
# detecting edges - less computationally
# expensive
prev_gray = cv.cvtColor(first_frame, cv.COLOR_BGR2GRAY)

# Creates an image filled with zero
# intensities with the same dimensions
# as the frame
mask = np.zeros_like(first_frame)

# Sets image saturation to maximum
mask[..., 1] = 255
flow_0_total = 0
flow_1_total = 0
magnitude_total = 0
angle_total = 0

try:
    while cap.isOpened():
        tic = time.time()
        # ret = a boolean return value from getting
        # the frame, frame = the current frame being
        # projected in the video
        ret, frame = cap.read()

        # Opens a new window and displays the input
        # frame
        cv.imshow("input", frame)

        # Converts each frame to grayscale - we previously
        # only converted the first frame to grayscale
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Calculates dense optical flow by Farneback method
        flow = cv.calcOpticalFlowFarneback(prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        flow_0_total += np.sum(flow[..., 0])
        flow_1_total += np.sum(flow[..., 1])

        # Computes the magnitude and angle of the 2D vectors
        magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])

        # Sum magnitude and angle overtime
        # magnitude_total += np.sum(magnitude)
        # angle_total += np.sum(angle)

        # Sets image hue according to the optical flow
        # direction
        mask[..., 0] = angle * 180 / np.pi / 2

        # Sets image value according to the optical flow
        # magnitude (normalized)
        mask[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)

        # Converts HSV to RGB (BGR) color representation
        rgb = cv.cvtColor(mask, cv.COLOR_HSV2BGR)

        # Opens a new window and displays the output frame
        cv.imshow("dense optical flow", rgb)

        # Updates previous frame
        prev_gray = gray

        # Frames are read by intervals of 1 millisecond. The
        # programs breaks out of the while loop when the
        # user presses the 'q' key
        if cv.waitKey(1) & 0xFF == ord("q"):
            break

        sys.stdout.write(f"FPS: {1 / (time.time() - tic):.2f}, M,A = {flow_0_total,flow_1_total}\r")
finally:
    # The following frees up resources and
    # closes all windows
    cap.release()
    cv.destroyAllWindows()
