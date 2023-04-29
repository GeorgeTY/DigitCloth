import cv2
import sys
import time
import os
import rospy
import numpy as np
from collections import namedtuple
from digit_device import Device
import multiprocessing as mp
from test_hd.msg import state


class Odometer:
    R, R_sum, R_selected = np.eye(2), np.eye(2), np.eye(2)
    T, T_sum, T_selected = np.zeros((2,)), np.zeros((2,)), np.zeros((2,))
    is_visualize = True
    is_report = False
    is_ros = False
    is_gpu = False

    def __init__(self):
        border = 20
        self.roi = [0 + border, 320 - border, 0 + border, 240 - border]
        self.feature_params = dict(maxCorners=100, qualityLevel=0.5, minDistance=5, blockSize=2)
        self.lk_params = dict(
            winSize=(20, 20), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1, 0.03)
        )
        self.cms = "Odometer_Init"

    def ros_callback(self, data):
        self.cms = data.state

    def ros_listener(self):
        rospy.Subscriber("Cloth_Maipulation_State", state, self.ros_callback)

    def ros_publisher(self):
        rospy.init_node("odometer", anonymous=True)
        rate = rospy.Rate(30)
        while True:
            rate.sleep()

    def crop_anything(self, thing, roi):
        return thing[roi[0] : roi[1], roi[2] : roi[3]]

    def reg_svd(self, X, Y):
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

        # check if the reflection is required
        if d:
            S[-1] = -S[-1]
            V[:, -1] = -V[:, -1]

        # create Rotation matrix R
        R = np.dot(V, Wt)

        # compute the translation vector T
        T = my - np.dot(R, mx)

        return R, T

    def odometer(self, frame_queue, result_queue, visualize_queue):
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        from matplotlib.transforms import Affine2D

        fig, ax = plt.subplots(figsize=(10, 10), num="Odometer | View Port")
        ax.set_xlim([0, 1000])
        ax.set_ylim([0, 1000])
        rect_cloth = patches.Rectangle((100, 100), 800, 800, color="green")
        rect_gelsight = patches.Rectangle((426, 410), 147, 180, alpha=0.5, color="red")
        ax.add_patch(rect_cloth)
        ax.add_patch(rect_gelsight)
        plt.pause(0.1)

        from gelsight import gs3drecon  # import outside will cause error

        model_file_path = "/home/ratiomiith/Development/DigitCloth/ros_workspace/src/get_height/scripts/"
        net_file_path = "nnmini.pt"
        net_path = os.path.join(model_file_path, net_file_path)

        gpuorcpu = "cuda" if self.is_gpu else "cpu"

        Device_Alias = namedtuple("GelSightMini", ["imgw", "imgh"])  # gelsight sdk requires this
        dev_alias = Device_Alias(240, 320)
        nn = gs3drecon.Reconstruction3D(gs3drecon.Finger.R15, dev_alias)
        nn.load_nn(net_path, gpuorcpu)

        frame_count = 0

        try:
            while True:
                tic = time.time()

                frame = frame_queue.get()
                frame_count += 1
                depthmap = nn.get_depthmap(frame, mask_markers=False)
                frame = self.crop_anything(frame, self.roi)  # Discard the Markers area
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if frame_count < 60:
                    prev_gray = gray
                    prev_corners = None
                    continue

                depth_mask = np.zeros_like(depthmap, dtype=np.uint8)
                depth_mask[depthmap < -1.5] = 1
                depth_mask = self.crop_anything(depth_mask, self.roi)

                contact_check = np.sum(depth_mask)

                if (
                    contact_check != 0 and prev_corners is None
                ):  # Set current frame as previous frame if contact is detected
                    prev_corners = cv2.goodFeaturesToTrack(prev_gray, mask=depth_mask, **self.feature_params)
                    prev_gray = gray
                elif contact_check != 0 and prev_corners is not None and len(prev_corners) > 2:
                    # Do odometer
                    curr_corners, _, _ = cv2.calcOpticalFlowPyrLK(
                        prev_gray, gray, prev_corners, None, **self.lk_params
                    )  # TODO: the tracking seems malfunctioning
                    curr_corners = curr_corners.squeeze()
                    prev_corners = prev_corners.squeeze()

                    # Registartion
                    R, T = self.reg_svd(prev_corners, curr_corners)

                    self.R_sum = np.matmul(self.R_sum, R)
                    self.T_sum = self.T_sum + T

                    # Visualize
                    M = Affine2D().from_values(
                        self.R_sum[0, 0],
                        self.R_sum[0, 1],
                        self.R_sum[1, 0],
                        self.R_sum[1, 1],
                        self.T_sum[0],
                        self.T_sum[1],
                    )
                    rect_gelsight.set_transform(M)
                    plt.pause(0.001)

                    if self.cms == "turnL_clock":
                        self.R_selected = np.matmul(self.R_selected, R)
                        self.T_selected = self.T_selected + T

                    self.visualize(
                        frame, prev_corners, curr_corners, depth_mask, visualize_queue
                    ) if self.is_visualize else None

                    prev_corners = cv2.goodFeaturesToTrack(prev_gray, mask=depth_mask, **self.feature_params)
                    prev_gray = gray
                else:
                    prev_corners = None

                packed_results = {"Process": "Odometer", "R": self.R_sum, "T": self.T_sum}
                result_queue.put(packed_results)

                sys.stdout.write(f"FPS: {1/(time.time()-tic):.2f}, Contact Check: {contact_check}\r")
        except KeyboardInterrupt:
            print("Odometer | KeyboardInterrupt")

    def visualize(self, frame, prev_corners, curr_corners, depth_mask, output_queue=None):
        canvas = np.zeros_like(frame)
        for i, (new, old) in enumerate(zip(curr_corners, prev_corners)):
            a, b = new.ravel()
            c, d = old.ravel()
            a, b, c, d = int(a), int(b), int(c), int(d)
            cv2.line(canvas, (a, b), (c, d), (0, 255, 0), 2)
            cv2.circle(canvas, (a, b), 5, (0, 0, 255), -1)
        frame_masked = frame.copy()
        frame_masked[depth_mask != 1] = [0, 0, 0]
        result = cv2.add(frame_masked, canvas)

        packed_frames = {
            "Process": "Odometer",
            "result": result,
        }
        output_queue.put(packed_frames)

    def run(self, frame_queue, result_queue, visualize_queue):
        self.processes = []

        if self.is_ros:
            self.processes.append(mp.Process(target=self.ros_listener))
            self.processes.append(mp.Process(target=self.ros_publisher))
        self.processes.append(mp.Process(target=self.odometer, args=(frame_queue, result_queue, visualize_queue)))

        [p.start() for p in self.processes]

    def stop(self):
        [p.terminate() for p in self.processes]


def main():
    gsmini = Device()
    odometer = Odometer()

    frame_queue = mp.Queue()
    result_queue = mp.Queue()
    visualize_queue = mp.Queue()

    gsmini.run(frame_queue)
    odometer.run(frame_queue, result_queue, visualize_queue)

    # Visualize
    try:
        while True:
            packed_results = result_queue.get()
            packed_frames = visualize_queue.get()
            if packed_frames["Process"] == "Odometer":
                cv2.imshow("Odometer | Tracking", packed_frames["result"])

            getKey = cv2.waitKey(1)
            if getKey & 0xFF == ord("q"):
                break
    finally:
        gsmini.stop()
        odometer.stop()


if __name__ == "__main__":
    main()
