import sys
import numpy as np
import cv2
import os
import rospy
import time
import curses
from datetime import datetime
from gelsight import gsdevice
from gelsight import gs3drecon
from get_height.msg import Height
from test_hd.msg import state
from matplotlib import pyplot as plt
import cProfile
import pstats
import io

pr = cProfile.Profile()
pr.enable()

cms = "test_hd_not_started"

old_stdout = sys.stdout
sys.stdout = open(os.devnull, "w")  # This prevents gelsight from printing to the console

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()


def reg_svd(X, Y):
    # X and Y are 2D matrices with the same number of rows
    # X is the matrix of points in the reference image
    # Y is the matrix of points in the current image
    # X and Y are of size N x 2
    # N is the number of points

    # compute the centroids of the points
    mx = np.mean(X, axis=0)
    my = np.mean(Y, axis=0)

    # subtract the centroids from the points
    X0 = X - mx
    Y0 = Y - my

    # compute the covariance matrix
    C = np.dot(np.transpose(X0), Y0)

    # compute the optimal rotation matrix
    # using the singular value decomposition
    V, S, Wt = np.linalg.svd(C)
    d = (np.linalg.det(V) * np.linalg.det(Wt)) < 0.0

    if d:
        S[-1] = -S[-1]
        V[:, -1] = -V[:, -1]

    # create Rotation matrix R
    R = np.dot(V, Wt)

    # compute the translation vector T
    T = my - np.dot(R, mx)

    return R, T


def report_result(R, T, R_sum, T_sum, R_selected, T_selected, tic):
    global frame_count, cms
    stdscr.addstr(0, 0, f"=== Time: {datetime.now()}, FrameNo.{frame_count}")
    stdscr.addstr(
        1,
        0,
        f"=== R: {np.array2string(R.flatten(), formatter={'float': lambda x: f'{x:.3f}'})}, "
        + f"T: {np.array2string(T.flatten(), formatter={'float': lambda x: f'{x:.3f}'})}",
    )
    stdscr.addstr(
        2,
        0,
        f"=== R_sum: {np.array2string(R_sum.flatten(), formatter={'float': lambda x: f'{x:.3f}'})}, "
        + f"T_sum: {np.array2string(T_sum.flatten(), formatter={'float': lambda x: f'{x:.3f}'})}",
    )
    stdscr.addstr(
        3,
        0,
        f"=== R_selected: {np.array2string(R_selected.flatten(), formatter={'float': lambda x: f'{x:.3f}'})}, "
        + f"T_selected: {np.array2string(T_selected.flatten(), formatter={'float': lambda x: f'{x:.3f}'})}",
    )
    stdscr.addstr(3, 0, f"=== State: {cms}")
    stdscr.addstr(4, 0, f"=== FPS: {1/(time.time()-tic)}")
    stdscr.refresh()


def callback(data):
    global cms
    cms = data.state


def listener():
    rospy.Subscriber("Cloth_Maipulation_State", state, callback)


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
feature_params = dict(maxCorners=100, qualityLevel=0.5, minDistance=5, blockSize=2)
lk_params = dict(winSize=(20, 20), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1, 0.03))

clear_corners = True
frame_count = 0
is_init = True
old_corners = None
old_center, new_center = None, None
x2, y2 = 0, 0
x1_sum, y1_sum = 0, 0
R, R_sum, R_selected = np.eye(2), np.eye(2), np.eye(2)
T, T_sum, T_selected = np.zeros((2,)), np.zeros((2,)), np.zeros((2,))


def array_show():
    tic = time.time()
    global flow_0_total, flow_1_total, magnitude_total, angle_total, gray, prev_gray, new_corner_flag, old_corners
    global frame_count, is_init, cms, x2, y2, R, T, R_sum, T_sum, R_selected, T_selected
    frame = dev.get_image(roi)
    frame_count += 1

    dm = nn.get_depthmap(frame, MASK_MARKERS_FLAG)

    if frame_count < 60:  # GelSight takes about 60 frames to warm up
        return None, x2, y2

    sys.stdout = old_stdout  # Allowing print statements to be printed on the terminal

    prev_gray = gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    height_mask = np.zeros_like(dm, dtype=np.uint8)
    height_mask[dm < -1.5] = 1
    # print(np.max(dm), np.min(dm), np.mean(dm)
    # sys.stdout.write(f"max: {np.max(dm)}, min: {np.min(dm)}, mean: {np.mean(dm)}\r")

    canvas = np.zeros_like(frame)
    height_sum = np.sum(height_mask)

    if height_sum != 0 and old_corners is None:
        old_corners = cv2.goodFeaturesToTrack(gray, mask=height_mask, **feature_params)
    elif height_sum != 0 and old_corners is not None and len(old_corners) > 2:
        update_corners, status, errors = cv2.calcOpticalFlowPyrLK(prev_gray, gray, old_corners, None, **lk_params)

        status = status.flat
        new_corners = update_corners[status == 1]
        old_corners = old_corners[status == 1]

        new_corners = new_corners.squeeze()
        old_corners = old_corners.squeeze()

        if len(new_corners) != len(old_corners):
            print("Error: new_corners and old_corners are not same length")
            return None, x2, y2

        # --- Registration here --- #
        # old_c = np.mean(old_corners, axis=1)
        # new_c = np.mean(new_corners, axis=1)

        # q1 = old_corners - old_c
        # q2 = new_corners - new_c

        # H = np.matmul(q1, q2.T)
        # U, X, V_t = np.linalg.svd(H)
        # R = np.matmul(V_t.T, U.T)
        # T = new_c - np.matmul(R, old_c)

        R, T = reg_svd(old_corners, new_corners)

        result = np.matmul(R, old_corners.T).T + T

        # sum R and T
        R_sum = np.matmul(R_sum, R)
        T_sum += T
        x2 = T_sum[0]
        y2 = T_sum[1]
        # select by cms
        if cms == "turnL_clock":
            R_selected = np.matmul(R_selected, R)
            T_selected += T
        # --- Registration here --- #

        # x_total = 0.0
        # y_total = 0.0

        for i, (new, old) in enumerate(zip(new_corners, old_corners)):
            a, b = new.ravel()
            c, d = old.ravel()
            a, b, c, d = int(a), int(b), int(c), int(d)
            cv2.line(canvas, (a, b), (c, d), (0, 255, 0), 2)
            ## calculate movement sum
            # x_total += a - c
            # y_total += b - d
            # cv2.circle(canvas, (a, b), 3, (0, 0, 255), -1)
        old_corners = cv2.goodFeaturesToTrack(gray, mask=height_mask, **feature_params)

        # sys.stdout.write(
        #     f"=== FPS: {float(1/(time.time()-tic)):4.2f}, x2: {x2:6.2f}, y2: {y2:6.2f}, CMS: {cms:15s} ===\r"
        # )

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

    report_result(R, T, R_sum, T_sum, R_selected, T_selected, tic)

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

    return height_array, x2, y2


def talker():
    global flow_0_total, flow_1_total, magnitude_total, angle_total
    pub = rospy.Publisher("Rdigit_height", Height, queue_size=10)

    rospy.init_node("Rdigit_height", anonymous=True)

    listener()

    rate = rospy.Rate(10)  # 10hz

    fig = plt.figure(1)
    fig.set_size_inches(6, 16)
    sample_count = 0

    while not rospy.is_shutdown():
        msg = Height()
        height_array, x2, y2 = array_show()
        if height_array is None:
            continue

        plt.subplot(211)
        plt.axis([0, 500, -200, 200])
        plt.title("OpticalFlow Result")
        plt.xlabel("sample count")
        plt.ylabel("x2(px)(Red)/y2(px)(Blue)")
        plt.scatter(sample_count, x2, s=2, c="r")
        plt.scatter(sample_count, y2, s=2, c="b")

        plt.subplot(212)
        plt.axis([-200, 200, -200, 200])
        plt.title("Estimated Movement")
        plt.xlabel("x(px)")
        plt.ylabel("y(px)")
        plt.scatter(x2, y2, s=2, c="b")

        # plt.pause(0.0001)
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
        curses.echo()
        curses.nocbreak()
        curses.endwin()

        pr.disable()
        s = io.StringIO()
        sortby = "cumulative"
        ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        ps.print_stats()
        pr.dump_stats("output/profile_" + now.strftime("%Y%m%d_%H%M%S") + ".prof")
