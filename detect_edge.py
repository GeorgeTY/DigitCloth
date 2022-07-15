import numpy as np
from global_params import *


def edgeDetection(tri, area_diff):
    for i, area in enumerate(area_diff):
        if area > ad_upper:
            return True
        elif area < ad_lower:
            return False


def main():
    tri = np.loadtxt("output/saved_tri.out", delimiter=",")
    pass


if __name__ == "__main__":
    main()
