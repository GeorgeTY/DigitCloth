import sys
import cv2
import numpy as np
from global_params import *
import digit_interface as Digit
from detect_blob import setDetectionParams, dotDetection


def connectDigit(intensity=8):
    digits = Digit.DigitHandler.list_digits()
    if len(digits) == 0:
        sys.exit("No Digit Found uwu")

    digit = Digit.Digit(digits[0]["serial"])
    digit.connect()
    digit.set_intensity(intensity)
    if ifVGA:
        digit.set_resolution(digit.STREAMS["VGA"])
    return digit


def main():
    digit = connectDigit()
    # digit.show_view()

    blobDetector = cv2.SimpleBlobDetector_create(setDetectionParams())

    while True:
        frm = digit.get_frame()
        hsv = cv2.cvtColor(frm, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, 0])
        upper = np.array([100, 100, 100])

        mask = cv2.inRange(hsv, lower, upper)
        res = cv2.bitwise_and(frm, frm, mask=mask)
        rev = abs(255 - res)

        keypoints, frm_with_keypoints = dotDetection(blobDetector, rev)

        cv2.imshow("Keypoints", frm_with_keypoints)
        cv2.moveWindow("Keypoints", 2020, 500)

        cv2.imshow("Preview", rev)
        cv2.moveWindow("Preview", 2020, 100)
        getKey = cv2.waitKey(1)
        if getKey == 27 or getKey == ord("q"):  # ESC or q
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
