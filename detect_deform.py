import cv2
import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt


class DelaunayTri:
    def __init__(self) -> None:
        self.is_first_frame = True
        self.is_timing = False
        self.is_visualize = True

    def __get_triangle_area(self, tp) -> float:
        return (
            abs(tp[0][0] * (tp[1][1] - tp[2][1]) + tp[1][0] * (tp[2][1] - tp[0][1]) + tp[2][0] * (tp[0][1] - tp[1][1]))
            / 2
        )

    def __get_area_diff(self, area_a, area_b) -> float:
        return area_b / area_a

    def get_visualized_frame(self):
        return self.frame_curr_with_tris

    def init(self, keypoints):
        self.tris, self.area_init = self.segment(keypoints)
        self.area_curr = np.zeros_like(self.area_init)

    def segment(self, keypoints):
        tris = Delaunay(keypoints)
        area = np.zeros(len(tris.simplices))
        X = (keypoints).astype(int)

        for i, simplex in enumerate(tris.simplices):
            area[i] = self.__get_triangle_area(keypoints[simplex])
        return tris, area

    def visualize(self, frame, keypoints, is_preview=False) -> None:
        self.frame_curr_with_tris = frame.copy()
        keypoints_int32 = np.int32(keypoints)
        if is_preview:
            for i, simplex in enumerate(self.tris.simplices):
                cv2.polylines(
                    self.frame_curr_with_tris, np.array([keypoints_int32[simplex]]), True, (0, 255, 255), 2, cv2.LINE_AA
                )

        else:
            for i, simplex in enumerate(self.tris.simplices):
                color = np.clip(
                    (127 - (self.area_diff[i] - 1) * 2 * 255, 0, 127 + (self.area_diff[i] - 1) * 2 * 255),
                    0,
                    255,
                )
                color = (int(color[0]), int(color[1]), int(color[2]))
                cv2.fillPoly(self.frame_curr_with_tris, np.array([keypoints_int32[simplex]]), tuple(color))
                cv2.polylines(
                    self.frame_curr_with_tris, np.array([keypoints_int32[simplex]]), True, (0, 255, 255), 1, cv2.LINE_AA
                )
                # cv2.putText(
                #     frame_temp,
                #     f"{self.area_diff[i]:.2f}",
                #     (int(keypoints[simplex][0][0]), int(keypoints[simplex][0][1])),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     0.5,
                #     (0, 0, 255),
                #     1,
                #     cv2.LINE_AA,
                # )

        cv2.imshow("Segment", self.frame_curr_with_tris)
        cv2.moveWindow("Segment", 600, 100)

    def preview(self, frame, keypoints):
        self.init(keypoints)
        frame_preview = frame.copy()
        keypoints_int32 = np.int32(keypoints)
        for i, simplex in enumerate(self.tris.simplices):
            cv2.polylines(
                frame_preview, np.int32(np.array([keypoints_int32[simplex]])), True, (0, 255, 255), 2, cv2.LINE_AA
            )
        cv2.imshow("Segment", frame_preview)
        cv2.moveWindow("Segment", 600, 100)

    def update(self, frame, keypoints):
        for i, simplex in enumerate(self.tris.simplices):
            self.area_curr[i] = self.__get_triangle_area(keypoints[simplex])
        self.area_diff = self.__get_area_diff(self.area_init, self.area_curr)
        self.visualize(frame, keypoints) if self.is_visualize else None


def main():
    points = np.loadtxt("./output/saved_X.out", delimiter=" ")
    delaunays = DelaunayTri()


if __name__ == "__main__":
    main()
