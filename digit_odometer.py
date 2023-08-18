import cv2
import sys
import time
import os
import rospy
import numpy as np
from datetime import datetime
from collections import namedtuple
from digit_device import Device
import multiprocessing as mp
from test_hd.msg import state
from get_height.msg import Odometer_result
from get_height.msg import Height


class Odometer:
    R, R_sum, R_selected = np.eye(2), np.eye(2), np.eye(2)
    T, T_sum, T_selected = np.zeros((2,)), np.zeros((2,)), np.zeros((2,))
    is_visualize = True
    is_plot = True
    is_report = False
    is_ros = True
    is_gpu = False

    R_log, T_log = [], []

    def __init__(self):
        border = 0
        self.roi = [0 + border, 240 - border, 0 + border, 320 - border]
        self.feature_params = dict(maxCorners=100, qualityLevel=0.5, minDistance=5, blockSize=2)
        self.lk_params = dict(
            winSize=(20, 20), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1, 0.03)
        )
        self.cms = "Odometer_Init"

    def __ros_callback(self, data):
        self.cms = data.state

    def __ros_listener(self):
        rospy.Subscriber("Cloth_Maipulation_State", state, self.__ros_callback)

    def __ros_publisher(self, height):
        pub = rospy.Publisher("current_height", Height, queue_size=10)
        pub_2 = rospy.Publisher("odometer_result", Odometer_result, queue_size=10)
        rospy.init_node("odometer", anonymous=True)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rospy.loginfo(f"Odometer | Publishing current height {height.value}")
            pub.publish(height.value)
            rospy.loginfo(f"Odometer | Publishing odometer result {self.T_shared}")
            pub_2.publish([self.T_shared[0], self.T_shared[1]])
            rate.sleep()

    def crop_anything(self, thing, roi):
        return thing[roi[0] : roi[1], roi[2] : roi[3]]

    def reg_svd(self, A, B):
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = np.dot(Vt.T, U.T)
        T = centroid_B.T - np.dot(R, centroid_A.T)
        return R, T

    def odometer(self, frame_queue, result_queue, visualize_queue, T_shared):
        if self.is_plot:
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches

            fig, ax = plt.subplots(figsize=(10, 10), num="Odometer | View Port")
            plt.title("Odometer | View Port")
            plt.xlabel("Pixels")
            plt.ylabel("Pixels")
            ax.set_xlim([-500, 500])
            ax.set_ylim([-500, 500])
            rect_cloth = patches.Rectangle((-400, -400), 800, 800, color="green")
            rect_gelsight = patches.Rectangle((-74, -90), 147, 180, alpha=0.5, color="red", rotation_point="center")
            ax.add_patch(rect_cloth)
            ax.add_patch(rect_gelsight)

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
                self.contact_check.value = contact_check

                if (
                    contact_check != 0 and prev_corners is None
                ):  # Set current frame as previous frame if contact is detected
                    prev_corners = cv2.goodFeaturesToTrack(prev_gray, mask=depth_mask, **self.feature_params)
                    prev_gray = gray
                elif contact_check != 0 and prev_corners is not None and len(prev_corners) > 2:
                    # Do odometer
                    curr_corners, _, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_corners, None, **self.lk_params)
                    curr_corners = curr_corners.squeeze()
                    prev_corners = prev_corners.squeeze()

                    # Registartion
                    R, T = self.reg_svd(prev_corners, curr_corners)

                    self.R_sum = np.matmul(self.R_sum, R)
                    self.T_sum = self.T_sum + T
                    T_shared[0] = self.T_sum[0]
                    T_shared[1] = self.T_sum[1]

                    if self.cms == "turnL_clock":
                        self.R_selected = np.matmul(self.R_selected, R)
                        self.T_selected = self.T_selected + T

                    if self.is_visualize:
                        self.visualize(frame, prev_corners, curr_corners, depth_mask, visualize_queue)
                    if self.is_plot:
                        # T_scaled = self.T_sum / 17.13  # T_sum counts in pixel, Visualize in mm
                        rect_gelsight.set_angle(np.rad2deg(np.arctan2(self.R_sum[1, 0], self.R_sum[0, 0])))
                        rect_gelsight.set_xy((-74 - self.T_sum[1], -90 - self.T_sum[0]))
                        center_gelsight = rect_gelsight.get_center()
                        # ax.scatter(center_gelsight[0], center_gelsight[1], s=2, c="yellow")  # This causes slow down
                        plt.pause(0.001)

                    prev_corners = cv2.goodFeaturesToTrack(prev_gray, mask=depth_mask, **self.feature_params)
                    prev_gray = gray
                else:
                    prev_corners = None

                packed_results = {"Process": "Odometer", "R": self.R_sum, "T": self.T_sum}
                result_queue.put(packed_results)

                sys.stdout.write(f"FPS: {1/(time.time()-tic):.2f}, Contact Check: {contact_check}, T: {self.T_sum}\r")
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
        manager = mp.Manager()
        self.T_shared = manager.Array("d", [0, 0])  # Shared memory for T
        self.contact_check = manager.Value("d", 0)  # Shared memory for contact check

        if self.is_ros:
            self.processes.append(mp.Process(target=self.__ros_listener))
            self.processes.append(mp.Process(target=self.__ros_publisher, args=(self.contact_check,)))
        self.processes.append(
            mp.Process(target=self.odometer, args=(frame_queue, result_queue, visualize_queue, self.T_shared))
        )

        [p.start() for p in self.processes]

    def save(self):
        self.T_log.append(np.array(self.T_shared))
        sys.stdout.write(f"\nSaved T: {self.T_shared}")

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

    time = datetime.now()

    # Visualize
    try:
        while True:
            # packed_results = result_queue.get()
            packed_frames = visualize_queue.get()
            if packed_frames["Process"] == "Odometer":
                cv2.imshow("Odometer | Tracking", packed_frames["result"])
                odometer.save()

            getKey = cv2.waitKey(1)
            if getKey & 0xFF == ord("q"):
                break
            elif getKey == 32:  # Spacebar
                odometer.save()
    finally:
        gsmini.stop()
        odometer.stop()
        time_str = time.strftime("%Y%m%d-%H%M%S")
        np.save("output/T_log_exp_odometer_" + time_str + ".npy", odometer.T_log)


if __name__ == "__main__":
    main()
