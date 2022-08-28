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

void moveCloth::moveGrab(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
{
    msg_L.X_Axis = msg_L.X_Axis - kGrabDeltaX;
    msg_R.X_Axis = msg_R.X_Axis + kGrabDeltaX;

    return;
}

void moveCloth::moveDetectEdge(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step) { return; }

void moveCloth::moveClothInwards(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step)
{
    switch (state_flag_)
    {
    case kInit:
        moveInit(msg_L, msg_R);
        state_flag_ = kGrab;
        break;
    case kGrab:
        moveGrab(msg_L, msg_R);
        state_flag_ = kRelease;
        break;
    case kRelease:
        moveInit(msg_L, msg_R);
        state_flag_ = kStop;
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
    float step_y = step * Deg2Rad * Radius;
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
