import cv2
import numpy as np

camera = cv2.VideoCapture(0)

es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
kernel = np.ones((5, 5), np.uint8)
background = None

while True:
    grabbed, frame_lwpCV = camera.read()

    gray_lwpCV = cv2.cvtColor(frame_lwpCV, cv2.COLOR_BGR2GRAY)
    gray_lwpCV = cv2.GaussianBlur(gray_lwpCV, (21, 21), 0)

    if background is None:
        background = gray_lwpCV
        continue

    diff = cv2.absdiff(background, gray_lwpCV)
    diff = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
    diff = cv2.dilate(diff, es, iterations=2)

    contours, hierarchy = cv2.findContours(
        diff.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    for c in contours:
        if cv2.contourArea(c) < 1000:
            continue
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(frame_lwpCV, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.imshow("contours", frame_lwpCV)
    cv2.imshow("diff", diff)

    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()
camera.release()
