# for Digit
ifRec = False
intensity = 10
# ifVGA = True
ifVGA = False

scale = 2

# for blobDetector
minThreshold = 0
maxThreshold = 255
minArea = 20
maxArea = 80
minArea_VGA = 20
maxArea_VGA = 100
minCircularity = 0.7
minConvexity = 0.8
minInertiaRatio = 0.01

# for CPD registration
cpd_tolerance = 1  ## 0.001
cpd_alpha = 0.002  ## 2
cpd_beta = 800  ## 2

# for edgeDetection
ed_method = 2  # 1:Linear Fit, 2:Line Clustering, 3:RANSAC
area_threshold = 40
ad_upper = 1.3
ad_lower = 0.7
ad_ratio = 1.15

# for linearRegression
lr_inputSize = 1
lr_outputSize = 1

# for RANSAC
ransac_iterations = 100
ransac_minInliers = 10
ransac_threshold = 0.05
