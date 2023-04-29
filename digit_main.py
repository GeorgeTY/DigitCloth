import cv2
import multiprocessing as mp
from digit_device import Device
from digit_markers import Markers
from digit_odometer import Odometer


def main():
    gsmini = Device()
    markers = Markers()
    odometer = Odometer()

    frame_queue = mp.Queue()
    result_queue = mp.Queue()
    visualize_queue = mp.Queue()

    gsmini.run(frame_queue)
    markers.run(frame_queue, result_queue,visualize_queue)
    odometer.run(frame_queue, result_queue,visualize_queue)

    # Visualize
    try:
        while True:
            packed_frames = visualize_queue.get()
            if packed_frames["Process"] == "Markers":
                cv2.imshow("Markers | Flow Vectors", packed_frames["frame_curr_with_flow_vectors"])
                cv2.imshow("Markers | Triangles", packed_frames["frame_curr_with_tris"])
                cv2.imshow("Markers | Edges", packed_frames["frame_curr_with_edges"])
            if packed_frames["Process"] == "Odometer":
                cv2.imshow("Odometer | Tracking", packed_frames["result"])

            getKey = cv2.waitKey(1)
            if getKey & 0xFF == ord("q"):
                break
    finally:
        gsmini.stop()
        markers.stop()
        odometer.stop()


if __name__ == "__main__":
    main()
