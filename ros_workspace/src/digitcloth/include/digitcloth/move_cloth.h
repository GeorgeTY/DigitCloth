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
#include "digitcloth/Edge_msg.h"

using namespace std;

class moveCloth
{
private:
    const float kDeg2Rad = M_PI / 180.0;
    const float kRad2Deg = 180.0 / M_PI;
    const float kInitPos_L_X = -28.0;
    const float kInitPos_L_Y = 120.0;
    const float kInitPos_L_Z = 0.0;
    const float kInitPos_R_X = -12.0;
    const float kInitPos_R_Y = 120.0;
    const float kInitPos_R_Z = 0.0;
    const float kRadius = 35.0;
    const float kFrictionAreaArc = 18.0;
    const float kFrictionAreaAngle = kFrictionAreaArc / kRadius; // in radians

    const float kGrabDeltaX = 4.0;     // Control the Gripping Force
    const float kRubUpperAmount = 0.5; // Control the Rubbing Amount
    const float kRubLowerAmount = 0.5;
    const float kRubDelta = 0.5;    // Control the Rubbing Speed
    const float kMoveDelta = 0.5;   // Control the Moving Speed
    const float kRotateDelta = 0.5; // Control the Rotating Speed
    enum stateFlags
    {
        kInit = 0,
        kGrab,
        kRelease,
        kRubRightUp,
        kRubRightDown,
        kRubRightReset,
        kInwards,
        kRotateL_NoMove_Clockwise,
        kRotateL_NoMove_CounterClockwise,
        kMoveL_UP,
        kMoveL_DOWN,
        kEdgeDectected,
        kStop
    };
    stateFlags state_flag_;

    float kRotateAngleLimit;
    float kMoveDistanceLimit;
    bool isEdgeDetected, isDistCalculated, isInfoPrinted;

    void moveL_up(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void moveR_up(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void moveL_down(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void moveR_down(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void rotateL_NoMove(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);

    void actionRotational(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void actionSliding(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);
    void actionRolling(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step);

public:
    void calcDistance(float distance);
    void moveInit(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R);
    void moveGrab(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R);
    void moveClothInwards(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float travel);
    void setState(int state);

    ros::Time tic, toc;

    int move_count, rub_count;
    class ClothEdge
    {
        bool isEdgeDetected;
        float P0;
        float P1;
        float P2;
    };
    ClothEdge edge_detected;
};

#endif