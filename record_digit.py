import time
from global_params import *
from connect_digit import connectDigit
import os
import cv2


def setVideoEncoder(fileName, ifVGA=False, scale=2):
    fps = videoFPS
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    videoOut = cv2.VideoWriter()
    tempName = "output/tmp.mp4"
    if ifVGA:
        videoOut.open(
            tempName,
            fourcc,
            fps,
            (480 * scale, 640 * scale),
        )
    else:
        videoOut.open(
            tempName,
            fourcc,
            fps,
            (240 * scale, 320 * scale),
        )
    return videoOut, tempName, fileName


def main():
    digit = connectDigit(intensity)
    for _ in range(60):  # Preheat the digit
        frm0 = digit.get_frame()

    timestr = time.strftime("%Y%m%d-%H%M%S")
    fileName = "output/recordDigit-{}.mp4".format(timestr)
    videoOut, videotempName, videofileName = setVideoEncoder(fileName, scale=1)

    while True:
        frm = digit.get_frame()

        cv2.imshow("recordDigit", frm)
        cv2.moveWindow("recordDigit", 2020, 100)

        videoOut.write(frm)

        getKey = cv2.waitKey(1)
        if getKey == ord("s"):
            os.rename(videotempName, videofileName)
            print("Video saved to output/")
            break
        if getKey == 27 or getKey == ord("q"):  # ESC or q
            break

    digit.disconnect()
    videoOut.release()
    os.remove(videotempName)


if __name__ == "__main__":
    main()
