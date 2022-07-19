import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay

points = np.array(
    [
        [0, 0],
        [1, 0],
        [0, 1],
        [1, 1],
        [2, 0],
        [2, 1],
        [3, 0],
        [3, 1],
        [0.5, 0.5],
        [1.5, 0.5],
        [2.5, 0.5],
        [3.5, 0.5],
    ]
)
tri = Delaunay(points)
print("simplicies:\n", tri.simplices)
print("neighbors:\n", tri.neighbors)

plt.triplot(points[:, 0], points[:, 1], tri.simplices)
plt.plot(points[:, 0], points[:, 1], "o")
plt.show()

"""
simplicies:
 [[ 8  2  0]
 [ 1  8  0]
 [ 8  3  2]
 [ 1  4  9]
 [10  4  6]
 [ 5  3  9]
 [ 5 10  7]
 [ 1  3  8]
 [ 3  1  9]
 [10  6 11]
 [ 7 10 11]
 [ 5  4 10]
 [ 4  5  9]]
neighbors:
 [[-1  1  2]
 [ 0 -1  7]
 [-1  0  7]
 [12  8 -1]
 [-1  9 11]
 [ 8 12 -1]
 [10 -1 11]
 [ 2  1  8]
 [ 3  5  7]
 [-1 10  4]
 [ 9 -1  6]
 [ 4  6 12]
 [ 5  3 11]]
"""
