#include "digitcloth/move_cloth.h"

using namespace std;

void moveCloth::moveInit(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
{
    msg_L.X_Axis = kInitPos_L_X;
    msg_L.Y_Axis = kInitPos_L_Y;
    msg_L.Z_Angle = kInitPos_L_Z;

    msg_R.X_Axis = kInitPos_R_X;
    msg_R.Y_Axis = kInitPos_R_Y;
    msg_R.Z_Angle = kInitPos_R_Z;

    return;
}

void moveCloth::calcDistance(float distance)
{
    if (distance > kFrictionAreaArc)
        kRotateAngleLimit = kFrictionAreaAngle * kRad2Deg;
    else if (0 < distance < kFrictionAreaArc)
        kRotateAngleLimit = distance / kRadius * kRad2Deg; // Return the angle in degrees
    else
        ROS_INFO("Distance Error");

    kMoveDistanceLimit = ((35 * kDeg2Rad - kRotateAngleLimit) * kRadius);
    return;
}

void moveCloth::moveGrab(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
{
    msg_L.X_Axis = kInitPos_L_X + kGrabDeltaX;
    msg_R.X_Axis = kInitPos_R_X - kGrabDeltaX;

    return;
}

void moveCloth::moveClothInwards(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float travel)
{
    if (!isDistCalculated)
    {

        isDistCalculated = true;
    }
    switch (state_flag_)
    {
    //运动到初始位置
    case kInit:
        if (!isInfoPrinted)
        {
            ROS_INFO("Move Cloth Inwards Init");
            isInfoPrinted = true;
            tic = ros::Time::now(); // Init Starting Time
        }

        moveInit(msg_L, msg_R);

        if (ros::Time::now() - tic > ros::Duration(5)) // Init Duration
        {
            state_flag_ = kGrab;
            isInfoPrinted = false;
        }
        break;
    //抓取
    case kGrab:
        if (!isInfoPrinted)
        {
            ROS_INFO("Move Cloth Inwards Grab");
            isInfoPrinted = true;
            tic = ros::Time::now(); // Grab Starting Time
        }

        moveGrab(msg_L, msg_R);

        if (ros::Time::now() - tic > ros::Duration(0.5)) // Grab Duration
        {
            state_flag_ = kRubRightDown;
            isInfoPrinted = false;
        }
        break;
    // Digit向下搓
    case kRubRightDown:
        if (!isInfoPrinted)
        {
            ROS_INFO("Rub Right DOWN: %d", rub_count);
            isInfoPrinted = true;
        }

        moveR_down(msg_L, msg_R, kRubDelta);
        ros::Duration(0.2).sleep();
        /* Digit Callback */

        if (msg_R.Y_Axis < kInitPos_R_Y - kRubLowerAmount)
        {
            state_flag_ = kRubRightUp;
            isInfoPrinted = false;
        }
        break;
    // Digit向上搓
    case kRubRightUp:
        if (!isInfoPrinted)
        {
            ROS_INFO("Rub Right UP: %d", rub_count);
            isInfoPrinted = true;
        }

        moveR_up(msg_L, msg_R, kRubDelta);
        if (isEdgeDetected)
        {
            ROS_INFO("in Func Edge Detected, Y: %.2f, EdgePos: %.2f %.2f %.2f", msg_R.Y_Axis, edge_detected.P0, edge_detected.P1, edge_detected.P2);
            ros::Duration(10).sleep();
        }
        ros::Duration(0.2).sleep();
        /* Digit Callback */

        if (msg_R.Y_Axis > kInitPos_R_Y + kRubUpperAmount)
        {
            state_flag_ = kRubRightReset;
            isInfoPrinted = false;
        }
        break;
    // Digit搓完后复位
    case kRubRightReset:
        if (!isInfoPrinted)
        {
            ROS_INFO("Rub Right RESET: %d", rub_count);
            isInfoPrinted = true;
        }

        moveR_up(msg_L, msg_R, -kRubDelta);
        ros::Duration(0.2).sleep();
        /* Digit Callback */

        if (msg_R.Y_Axis <= kInitPos_R_Y)
        {
            state_flag_ = kRotateL_NoMove_Clockwise;
            isInfoPrinted = false;
            ros::Duration(5).sleep();
        }
        break;
    // L指顺时针旋转，接触位置不变
    case kRotateL_NoMove_Clockwise:
        if (!isInfoPrinted)
        {
            ROS_INFO("Rotate Left No Move Clockwise: %f", msg_L.Z_Angle);
            isInfoPrinted = true;
        }

        rotateL_Move(msg_L, msg_R, kRotateDelta);
        ROS_INFO("Rotate Left No Move Clockwise: %f, %f", msg_L.Z_Angle, kRotateDelta);
        ros::Duration(0.2).sleep();

        if (msg_L.Z_Angle >= kInitPos_L_Z + kRotateAngleLimit / 2)
        {
            state_flag_ = kMoveL_UP;
            isInfoPrinted = false;
        }
        break;
    // L指向上移动
    case kMoveL_UP:
        if (!isInfoPrinted)
        {
            ROS_INFO("Move Left UP: %f", msg_L.Y_Axis);
            isInfoPrinted = true;
        }

        moveL_up(msg_L, msg_R, kMoveDelta);

        if (msg_L.Y_Axis >= kInitPos_L_Y + kMoveDistanceLimit)
        {
            state_flag_ = kRotateL_NoMove_CounterClockwise;
            isInfoPrinted = false;
        }
        break;
    case kRotateL_NoMove_CounterClockwise:
        if (!isInfoPrinted)
        {
            ROS_INFO("Rotate Left No Move CounterClockwise: %f", msg_L.Z_Angle);
            isInfoPrinted = true;
        }

        rotateL_NoMove(msg_L, msg_R, -kRotateDelta);

        if (msg_L.Z_Angle <= kInitPos_L_Z)
        {
            state_flag_ = kGrab;
            isInfoPrinted = false;
        }
        break;
    case kStop:
        break;
    }

    return;
}

void moveCloth::actionRotational(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    msg_L.Z_Angle = msg_L.Z_Angle + step;

    return;
}

void moveCloth::actionSliding(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    msg_L.Y_Axis = msg_L.Y_Axis + step;
}

void moveCloth::actionRolling(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    float step_y = step * kDeg2Rad * kRadius;
    msg_L.Z_Angle = msg_L.Z_Angle + step;
    msg_L.Y_Axis = msg_L.Y_Axis + step_y;

    return;
}

void moveCloth::moveL_up(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    msg_L.Y_Axis = msg_L.Y_Axis + step;
}

void moveCloth::moveR_up(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    msg_R.Y_Axis = msg_R.Y_Axis + step;
}

void moveCloth::moveL_down(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    msg_L.Y_Axis = msg_L.Y_Axis - step;
}

void moveCloth::moveR_down(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    msg_R.Y_Axis = msg_R.Y_Axis - step;
}

void moveCloth::rotateL_Move(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    msg_L.Z_Angle = msg_L.Z_Angle + step;
    return;
}

void moveCloth::rotateL_NoMove(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    msg_L.Z_Angle = msg_L.Z_Angle + step;
    msg_L.Y_Axis = msg_L.Y_Axis + step * kDeg2Rad * kRadius;
    return;
}
