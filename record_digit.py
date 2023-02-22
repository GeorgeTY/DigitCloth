import time
from global_params import *
from connect_gsmini import connectGSmini
import os
import cv2


def setVideoEncoder(fileName):
    fps = videoFPS
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    videoOut = cv2.VideoWriter()
    tempName = "output/tmp.mp4"
    videoOut.open(
        tempName,
        fourcc,
        fps,
        (320, 240),
    )
    return videoOut, tempName, fileName


def main():
    gsmini = connectGSmini()

    for _ in range(5):
        _ = gsmini.get_image((384, 288))

    timestr = time.strftime("%Y%m%d-%H%M%S")
    fileName = "output/recordDigit-{}.mp4".format(timestr)
    videoOut, videotempName, videofileName = setVideoEncoder(fileName)

    try:
        while True:
            frm = gsmini.get_image((384, 288))

            cv2.imshow("recordDigit", frm)
            cv2.moveWindow("recordDigit", 220, 100)

            videoOut.write(frm)

            getKey = cv2.waitKey(1)
            if getKey == 27 or getKey == ord("q") or getKey == ord("s"):  # ESC or q or s
                break
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        cv2.destroyAllWindows()
        gsmini.stop_video()
        videoOut.release()
        if getKey == ord("s"):
            print("Video saved to output/")
            os.rename(videotempName, videofileName)
        else:
            os.remove(videotempName)


if __name__ == "__main__":
    main()
