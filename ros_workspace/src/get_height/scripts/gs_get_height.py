import sys
import numpy as np
import cv2
import os
import rospy
import time
from scipy import ndimage
from datetime import datetime
from gelsight import gsdevice
from gelsight import gs3drecon
from get_height.msg import Height
from test_hd.msg import state
from matplotlib import pyplot as plt

cms = "test_hd_not_started"


def callback(data):
    global cms
    cms = data.state


def listener():
    rospy.Subscriber("Cloth_Maipulation_State", state, callback)


def get_diff_img(img1, img2):
    return np.clip((img1.astype(int) - img2.astype(int)), 0, 255).astype(np.uint8)


def get_diff_img_2(img1, img2):
    return (img1 * 1.0 - img2) / 255.0 + 0.5


GPU = False
MASK_MARKERS_FLAG = False
FIND_ROI = False

path = "/home/ratiomiith/Development/DigitCloth/ros_workspace/src/get_height/scripts/"
finger = gsdevice.Finger.MINI
mmpp = 0.0625

cam_id = gsdevice.get_camera_id("GelSight Mini")
dev = gsdevice.Camera(finger, cam_id)
net_file_path = "nnmini.pt"

dev.connect()

""" Load neural network """
model_file_path = path
net_path = os.path.join(model_file_path, net_file_path)
print("net path = ", net_path)

if GPU:
    gpuorcpu = "cuda"
else:
    gpuorcpu = "cpu"

nn = gs3drecon.Reconstruction3D(gs3drecon.Finger.R15, dev)
net = nn.load_nn(net_path, gpuorcpu)

f0 = dev.get_raw_image()
roi = (0, 0, f0.shape[1], f0.shape[0])
f0 = dev.get_image(roi)

# vis3d = gs3drecon.Visualize3D(dev.imgh, dev.imgw, "", mmpp)

mask = np.zeros_like(f0)

# Sets image saturation to maximum
mask[..., 1] = 255
flow_0_total = 0
flow_1_total = 0
magnitude_total = 0
angle_total = 0

gray, prev_gray = cv2.cvtColor(f0, cv2.COLOR_BGR2GRAY), cv2.cvtColor(f0, cv2.COLOR_BGR2GRAY)
feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=1, blockSize=2)  # 100, 0.3, 7, 7
lk_params = dict(winSize=(20, 20), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1, 0.03))

clear_corners = True
frame_count = 0
is_init = True
old_corners = None
old_center, new_center = None, None
x2, y2 = 0, 0
x1_sum, y1_sum = 0, 0
count = 0


def array_show():
    tic = time.time()
    global flow_0_total, flow_1_total, magnitude_total, angle_total, gray, prev_gray, new_corner_flag, old_corners, frame_count, is_init, old_center, new_center, x2, y2, x1_sum, y1_sum, count, cms
    frame = dev.get_image(roi)
    frame_count += 1

    dm = nn.get_depthmap(frame, MASK_MARKERS_FLAG)

    if frame_count < 60:
        return None, x2

    prev_gray = gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    # flow_0_total += np.sum(flow[..., 0])
    # flow_1_total += np.sum(flow[..., 1])

    # magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    # mask[..., 0] = angle * 180 / np.pi / 2
    # mask[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)
    # rgb = cv2.cvtColor(mask, cv2.COLOR_HSV2BGR)
    # rgb = cv2.putText(
    #     rgb, "OF " + f"{flow_0_total:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA
    # )

    height_mask = np.zeros_like(dm, dtype=np.uint8)
    height_mask[dm < -1.5] = 1
    # print(np.max(dm), np.min(dm), np.mean(dm)
    # sys.stdout.write(f"max: {np.max(dm)}, min: {np.min(dm)}, mean: {np.mean(dm)}\r")

    canvas = np.zeros_like(frame)
    height_sum = np.sum(height_mask)

    if height_sum != 0 and old_corners is None:
        old_corners = cv2.goodFeaturesToTrack(
            gray, mask=height_mask, maxCorners=50, qualityLevel=0.2, minDistance=7, blockSize=3
        )
        if old_corners is not None:
            old_center = (np.mean(old_corners[:, :, 0]), np.mean(old_corners[:, :, 1]))
    elif height_sum != 0 and old_corners is not None:
        update_corners, status, errors = cv2.calcOpticalFlowPyrLK(prev_gray, gray, old_corners, None, **lk_params)

        status = status.flat
        new_corners = update_corners[status == 1]
        old_corners = old_corners[status == 1]

        x_total = 0.0
        y_total = 0.0

        for i, (new, old) in enumerate(zip(new_corners, old_corners)):
            a, b = new.ravel()
            c, d = old.ravel()
            a, b, c, d = int(a), int(b), int(c), int(d)
            cv2.line(canvas, (a, b), (c, d), (0, 255, 0), 2)
            ## calculate movement sum
            x_total += a - c
            y_total += b - d
            # cv2.circle(canvas, (a, b), 3, (0, 0, 255), -1)
        if old_corners is not None:
            new_center = (np.mean(new_corners[:, :, 0]), np.mean(new_corners[:, :, 1]))
            old_center = (np.mean(old_corners[:, :, 0]), np.mean(old_corners[:, :, 1]))
        old_corners = cv2.goodFeaturesToTrack(
            gray, mask=height_mask, maxCorners=50, qualityLevel=0.2, minDistance=7, blockSize=3
        )

        if len(new_corners) > 0:
            x_bar = x_total / len(new_corners)
            y_bar = y_total / len(new_corners)
        else:
            x_bar = -0.45
            y_bar = -0.5

        x1 = new_center[0] - old_center[0]
        y1 = new_center[1] - old_center[1]
        # x2 += x_bar - x1
        # y2 += y_bar - y1
        x2 += x_bar + 0.45  # Drift correction
        y2 += y_bar + 0.5
        x1_sum += x1
        y1_sum += y1

        count += 1
        x1_bar = x1_sum / (count)
        y1_bar = y1_sum / (count)

        # print(f"x: {x_bar:6.2f}, y: {y_bar:6.2f}, x2: {x2:6.2f}, y2: {y2:6.2f}, x1: {x1_bar:6.2f}, y1: {y1_bar:6.2f}")
        sys.stdout.write(
            f"=== FPS: {float(1/(time.time()-tic)):4.2f}, x2: {x2:6.2f}, y2: {y2:6.2f}, CMS: {cms:15s} ===\r"
        )

        # if corners is not None:

        #     corners = np.int0(corners)
        #     for i in corners:
        #         x, y = i.ravel()
        #         cv2.circle(canvas, (x, y), 3, (0, 0, 255), -1)
        #     corners = None
        frame_masked = frame.copy()
        frame_masked[height_mask != 1] = [0, 0, 0]
        result = cv2.add(frame_masked, canvas)
        cv2.imshow("result", result)
    else:
        old_corners = None

    cv2.imshow("frame", frame)
    # cv2.imshow("dense optical flow", rgb)
    # """ Display the results """
    # vis3d.update(dm)

    width = dm.shape[1]
    height = dm.shape[0]
    lin_num = 3
    col_num = 6

    height_array = np.ones((lin_num, col_num))

    cols = np.arange(0, width + 0.01, width // lin_num)
    lins = np.arange(0, height + 0.01, height // col_num)
    cols = cols.astype(np.int16)
    lins = lins.astype(np.int16)

    for i in range(cols.shape[0] - 1):
        for j in range(lins.shape[0] - 1):
            slice = dm[cols[i] : cols[i + 1], lins[j] : lins[j + 1]]
            mean = slice.mean()

            if mean > -0.1:
                height_array[i][j] = 0.0
            else:
                height_array[i][j] = mean

    return height_array, x2


def talker():
    global flow_0_total, flow_1_total, magnitude_total, angle_total
    pub = rospy.Publisher("Rdigit_height", Height, queue_size=10)

    rospy.init_node("Rdigit_height", anonymous=True)

    listener()

    rate = rospy.Rate(10)  # 10hz

    plt.axis([0, 500, -100, 200])
    plt.title("OpticalFlow Result")
    plt.xlabel("sample count")
    plt.ylabel("x2(px)")
    sample_count = 0

    while not rospy.is_shutdown():
        msg = Height()
        height_array, x2 = array_show()
        if height_array is None:
            continue

        plt.scatter(sample_count, x2, s=2, c="r")
        plt.pause(0.0001)
        sample_count += 1

        msg.which_digit = "Digit_R"
        msg.height_data0[0] = height_array[0][5]
        msg.height_data0[1] = height_array[1][5]
        msg.height_data0[2] = height_array[2][5]
        msg.height_data1[0] = height_array[0][4]
        msg.height_data1[1] = height_array[1][4]
        msg.height_data1[2] = height_array[2][4]
        msg.height_data2[0] = height_array[0][3]
        msg.height_data2[1] = height_array[1][3]
        msg.height_data2[2] = height_array[2][3]
        msg.height_data3[0] = height_array[0][2]
        msg.height_data3[1] = height_array[1][2]
        msg.height_data3[2] = height_array[2][2]
        msg.height_data4[0] = height_array[0][1]
        msg.height_data4[1] = height_array[1][1]
        msg.height_data4[2] = height_array[2][1]
        msg.height_data5[0] = height_array[0][0]
        msg.height_data5[1] = height_array[1][0]
        msg.height_data5[2] = height_array[2][0]

        pub.publish(msg)

        getKey = cv2.waitKey(1)
        if getKey == 27 or getKey == ord("q"):
            break

        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        now = datetime.now()
        filename = "./output/ofresult_x2_" + now.strftime("%Y%m%d_%H%M%S") + ".png"
        plt.savefig(filename)
        cv2.destroyAllWindows()
        dev.stop_video()
