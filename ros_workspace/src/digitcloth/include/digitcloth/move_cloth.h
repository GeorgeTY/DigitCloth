#ifndef MOVE_CLOTH
#define MOVE_CLOTH
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include <vector>
#include <string>
#include "hd_servo/EndPos.h"

using namespace std;

class moveCloth
{
private:
    const float kInitPos_L_X = -28.0;
    const float kInitPos_L_Y = 120.0;
    const float kInitPos_L_Z = 0.0;
    const float kInitPos_R_X = -12.0;
    const float kInitPos_R_Y = 120.0;
    const float kInitPos_R_Z = 0.0;
    const float Radius = 35.0;
    const float Deg2Rad = M_PI / 180.0;

    const float kGrabDeltaX = 4.0; // Control the Gripping Force
    enum stateFlags
    {
        kInit = 0,
        kGrab,
        kRelease,
        kInwards,
        kDetectEdge,
        kStop
    };
    stateFlags state_flag_;
    class ClothEdge
    {
        float P0;
        float P1;
        float P2;
    };

    void moveL_up(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void moveR_up(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void moveL_down(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void moveR_down(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void rotateL_NoMove(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void rotateR_NoMove(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);

    void actionRotational(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void actionSliding(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void actionRolling(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);

public:
    void moveInit(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R);
    void moveGrab(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R);
    void moveDetectEdge(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void moveClothInwards(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void setState(int state);

    int move_count = 0;
    bool isEdgeDetected;
    ClothEdge edge_detected;
};

#endif