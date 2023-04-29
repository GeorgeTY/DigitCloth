import cv2
import multiprocessing as mp
from gelsight import gsdevice


class Device:
    def __init__(self, cam_id=None) -> None:
        cam_id = gsdevice.get_camera_id("GelSight Mini") if cam_id is None else cam_id
        self.dev = gsdevice.Camera(gsdevice.Finger.MINI, cam_id)
        self.dev.connect()

    def get_frame(self, frame_queue):
        # use self.dev.get_image((320, 240)) to get a 320x240 image if you want
        try:
            while True:
                frame = self.dev.get_image((320, 240))
                frame_queue.put(frame)
        finally:
            self.dev.stop_video()

    def run(self, frame_queue):
        """Starts the device and the process that gets the frame from the device
        frame_queue is a multiprocessing.Queue()
        that is used to pass the frame to the other processes"""
        self.processes = []
        self.processes.append(mp.Process(target=self.get_frame, args=(frame_queue,)))

        [p.start() for p in self.processes]

    def stop(self):
        [p.terminate() for p in self.processes]


if __name__ == "__main__":
    gsmini = Device()
    frame_queue = mp.Queue()
    gsmini.run(frame_queue)
    try:
        while True:
            frame = frame_queue.get()
            cv2.imshow("gsmini", frame)
            getKey = cv2.waitKey(1)
            if getKey == 27 or getKey == ord("q"):  # ESC or q
                break
    finally:
        gsmini.stop()
        gsmini.dev.stop_video()
        cv2.destroyAllWindows()

# TODO: Make sure both of the processes gets the same frame
