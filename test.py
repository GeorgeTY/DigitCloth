import cv2
from connect_gsmini import connectGSmini


feature_params = dict(maxCorners=100, qualityLevel=0.5, minDistance=5, blockSize=2)


def main():
    gsmini = connectGSmini()

    try:
        while True:
            frame = gsmini.get_image((320, 240))

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            old_corners = cv2.goodFeaturesToTrack(gray, **feature_params)

            frm_with_keypoints = frame.copy()

            for corner in old_corners:
                x, y = corner.ravel()
                x, y = int(x), int(y)
                cv2.circle(frm_with_keypoints, (x, y), 3, 255, -1)

            # Show the image
            cv2.imshow("gsmini", frm_with_keypoints)
            getKey = cv2.waitKey(1)
            if getKey == 27 or getKey == ord("q"):  # ESC or q
                break
    except KeyboardInterrupt:
        gsmini.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
