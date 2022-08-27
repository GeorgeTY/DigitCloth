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

void moveCloth::moveDetectEdge(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float step) {}