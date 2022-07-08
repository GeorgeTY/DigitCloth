import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from pycpd import DeformableRegistration
from detect_blob import dotDetection
from global_params import *


def dotRegistration(keypoints_a, keypoints_b):
    X = np.array([keypoint.pt for keypoint in keypoints_a])
    Y = np.array([keypoint.pt for keypoint in keypoints_b])

    TY, (G, W, P) = DeformableRegistration(
        **{"X": X, "Y": Y}, alpha=alpha, beta=beta
    ).register()  ## CPD registration

    return X, Y, TY, G, W, P


def getRegParam(self):
    return self.G, self.W, self.P


def dotMatching(X, Y, TY, P, Frm0, Frm, scale=2):
    Frm_dot_movement = cv2.addWeighted(Frm, 0.65, Frm0, 0.35, 0)
    Frm_dot_movement = cv2.resize(
        Frm_dot_movement,
        (scale * Frm_dot_movement.shape[1], scale * Frm_dot_movement.shape[0]),
        interpolation=cv2.INTER_AREA,
    )

    #### Dot matching using P ####
    dotPair = np.zeros_like(P)
    for i in range(np.shape(P)[0]):
        dotPair[i][np.argmax(P[i][:])] = 1

    # dotPair = np.zeros_like(P)
    # while True:
    #     continueFlag = False
    #     for i in range(np.shape(P)[0]):
    #         argmax_j = np.argmax(P[i][:])
    #         # dotPairProb[i][argmax_j] = P[i][argmax_j]
    #         if np.count_nonzero(dotPair[:][argmax_j]) > 0:
    #             j = argmax_j
    #             argmax_i = np.argmax(dotPair[:][argmax_j])
    #             if P[i][j] > dotPair[argmax_i][j]:
    #                 dotPair[i][j] = P[i][j]
    #                 dotPair[argmax_i][j] = 0
    #                 P[argmax_i][j] = 0
    #             else:
    #                 P[i][j] = 0
    #         else:
    #             dotPair[i][argmax_j] = P[i][argmax_j]

    #     for i in range(np.shape(P)[0]):
    #         if np.count_nonzero(dotPair[i][:]) > 1:
    #             continueFlag = True
    #             break
    #     for j in range(np.shape(P)[1]):
    #         if np.count_nonzero(dotPair[:][j]) > 1:
    #             continueFlag = True
    #             break
    #     if not continueFlag:
    #         break
    #### End of Dot matching using P ####

    #### Dot matching using TY distance ####
    # distance = np.zeros((len(X), len(TY)))
    # dotPair = np.zeros((len(X), len(TY))).astype(bool)

    # for i in range(len(X)):
    #     for j in range(len(TY)):
    #         distance[i][j] = np.linalg.norm(X[i] - TY[j])
    #     argmin_j = np.argmin(distance[i][:])

    #     # ## Temporary Implementation: Prevent the dot from being registered twice
    #     # while dotPair[i][argmin_j] == 1:
    #     #     distance[i][argmin_j] = np.inf
    #     #     argmin_j = np.argmin(distance[i][:])
    #     # ## Temporary Implementation end

    #     dotPair[i][argmin_j] = 1
    #### End of Dot matching using TY distance ####

    #### Dot matching using P . dotPair ####
    # dotPair = calcMatrixM(P) ## Too slow
    #### End of Dot matching using P . dotPair ####

    # np.savetxt("output/saved_dotPair.out", dotPair, delimiter=",")
    for i in range(np.shape(P)[0]):
        for j in range(np.shape(P)[1]):
            if dotPair[i][j] > 0:
                cv2.arrowedLine(
                    Frm_dot_movement,
                    (int(X[j][0]) * scale, int(X[j][1]) * scale),
                    (
                        int(Y[i][0] * scale),
                        int(Y[i][1]) * scale,
                    ),
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                break

    return Frm_dot_movement, dotPair


## Add P output to existing function
DeformableRegistration.get_registration_parameters = getRegParam


def main():

    tic = time.time()
    Frm = cv2.imread("./pics/marker_movement/Frm.png")
    # Frm = cv2.imread("./pics/marker_movement/Frm_Lack.png")
    Frm0 = cv2.imread("./pics/marker_movement/Frm0.png")
    blobDetector = cv2.SimpleBlobDetector_create()
    keypoints_Frm, Frm_with_keypoints = dotDetection(blobDetector, Frm)
    keypoints_Frm0, Frm0_with_keypoints = dotDetection(blobDetector, Frm0)

    X, Y, TY, G, W, P = dotRegistration(keypoints_Frm0, keypoints_Frm)
    Frm_dot_movement, dotPair = dotMatching(X, Y, TY, P, Frm0, Frm)
    # np.savetxt("./output/saved_TY.out", TY, delimiter=",")
    # np.savetxt("./output/saved_G.out", G, delimiter=",")
    # np.savetxt("./output/saved_W.out", W, delimiter=",")
    np.savetxt("./output/saved_X.out", X, delimiter=" ")
    # np.savetxt("./output/saved_P.out", P * 100, delimiter=",", fmt="%d")

    tri = Delaunay(X)
    for simplex in tri.simplices:
        simplex = np.append(simplex, simplex[0])
        print(simplex)
        for i in range(len(simplex) - 1):
            cv2.line(
                Frm_dot_movement,
                (int(X[simplex[i]][0] * 2), int(X[simplex[i]][1] * 2)),
                (int(X[simplex[i + 1]][0] * 2), int(X[simplex[i + 1]][1] * 2)),
                (0, 255, 255),
                2,
            )

    # Calculate the distance
    # distance = np.zeros((len(P[0]), len(P[1])))
    # for i in range(len(X)):
    #     for j in range(len(TY)):
    #         if dotPair[i][j] > 0:
    #             distance[i][j] = np.linalg.norm(X[i] - TY[j])
    #         else:
    #             distance[i][j] = np.inf

    # print("Distance:")
    # for i in range(len(X)):
    #     for j in range(len(Y)):
    #         if dotPair[i][j] > 0:
    #             print("({},{}), ".format(i, j), distance[i][j])
    #             cv2.putText(
    #                 Frm_dot_movement,
    #                 "%.4f" % P[j][i],
    #                 (int(X[i][0]) * 2 - 5, int(X[i][1]) * 2 - 5),
    #                 cv2.FONT_HERSHEY_SIMPLEX,
    #                 0.5,
    #                 (0, 0, 255),
    #                 1,
    #                 cv2.LINE_AA,
    #             )
    #             pass

    cv2.imshow("Dot Movement", Frm_dot_movement)
    cv2.moveWindow("Dot Movement", 100, 100)

    toc = time.time()
    cv2.waitKey(0)
    print("Time: {}".format(toc - tic))

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
