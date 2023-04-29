import cv2
import numpy as np

# 创建一个正方形
square = np.zeros((300, 300), np.uint8)
square[50:250, 50:250] = 255

# 定义旋转矩阵和平移矩阵
R = cv2.getRotationMatrix2D((150, 150), 30, 1)
T = np.float32([[1, 0, 50], [0, 1, -30]])

# 进行刚性变换
square_rotated = cv2.warpAffine(square, R, (300, 300))
square_transformed = cv2.warpAffine(square_rotated, T, (300, 300))

# 显示结果
cv2.imshow("Original Square", square)
cv2.imshow("Rotated Square", square_rotated)
cv2.imshow("Transformed Square", square_transformed)
cv2.waitKey(0)
cv2.destroyAllWindows()
