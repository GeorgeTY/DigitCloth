import sys
import numpy as np
import cv2
import time
from gelsight import gsdevice
from ultralytics import YOLO


def get_diff_img(img1, img2):
    return np.clip((img1.astype(int) - img2.astype(int)), 0, 255).astype(np.uint8)


def get_diff_img_2(img1, img2):
    return (img1 * 1.0 - img2) / 255.0 + 0.5


GPU = False
MASK_MARKERS_FLAG = True
FIND_ROI = False

path = "/home/ratiomiith/Development/DigitCloth/ros_workspace/src/get_height/scripts/"
finger = gsdevice.Finger.MINI
mmpp = 0.0625

cam_id = gsdevice.get_camera_id("GelSight Mini")
dev = gsdevice.Camera(finger, cam_id)

dev.connect()

model = YOLO("yolov8n-seg.pt")

f0 = dev.get_raw_image()
roi = (0, 0, f0.shape[1], f0.shape[0])

if __name__ == "__main__":
    try:
        while True:
            tic = time.time()
            frame = dev.get_image(roi)
            cv2.imshow("frame", frame)

            results = model(frame)
            results_plotted = results[0].plot()
            cv2.imshow("results", results_plotted)

            getKey = cv2.waitKey(1)
            if getKey == ord("q"):
                break

            sys.stdout.write("\rFPS: %f" % (1 / (time.time() - tic)))

    finally:
        cv2.destroyAllWindows()
        dev.stop_video()
