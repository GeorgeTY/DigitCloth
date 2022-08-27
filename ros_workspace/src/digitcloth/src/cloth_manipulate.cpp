#include "digitcloth/cloth_manipulate.h"

using namespace std;

void Move_cloth::Init(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
{
    flag_ = init_position;
    init_begin = ros::Time::now();
    init_duration = ros::Duration(6);

    delta_theta = w * (1 / ((float)frequency));
    delta_ythreshold = ((theta_threshold / 2) * deg2rad * Radius) + (theta_threshold / 2 / delta_theta) * Modify; //默认是对中位置开始
    start_angle = InitL_PosAngle;

    E1 = 0;
    E2 = 0;
    E3 = 0;
    init_sum_height = 0.0;

    count = 0;

    msg_L.X_Axis = InitL_PosX;
    msg_L.Y_Axis = InitL_PosY;
    msg_L.Z_Angle = InitL_PosAngle;
    msg_R.X_Axis = InitR_PosX;
    msg_R.Y_Axis = InitR_PosY;
    msg_R.Z_Angle = InitR_PosAngle;

    return;
}

void Move_cloth::Set_MoveDistance_in(float distance, hd_servo::EndPos &msg_L)
{

    /*经过测试，由于末端开始滑动，实际的运动距离并不能完全满足设定，需要进行修正*/
    if (distance > theta_len)
    {
        // ROS_INFO("Cloth Move_Distabce Too far !!!");
        start_angle = -(theta_miu / 2);
    }
    else if (0 < distance < theta_len)
    {
        start_angle = (theta_miu / 2) - (distance / Radius) * rad2deg;
    }
    else
    {
        ROS_INFO("Error");
    }
    float delta = (theta_threshold / 2 - start_angle);
    delta_ythreshold = (delta * deg2rad * Radius) + (delta / delta_theta) * Modify;

    return;
}
void Move_cloth::Set_MoveDistance_out(float distance, hd_servo::EndPos &msg_L)
{
    /*经过测试，由于末端开始滑动，实际的运动距离并不能完全满足设定，需要进行修正*/
    if (distance > theta_len)
    {
        // ROS_INFO("Cloth Move_Distabce Too far !!!");
        start_angle = (theta_miu / 2);
    }
    else if (0 < distance < theta_len)
    {
        start_angle = (distance / Radius) * rad2deg - (theta_miu / 2);
    }
    else
    {
        ROS_INFO("Error");
    }
    float delta = (theta_threshold / 2 + start_angle);
    delta_ythreshold = (delta * deg2rad * Radius) + (delta / delta_theta) * Modify;

    init_begin = ros::Time::now();

    return;
}

void Move_cloth::Move_Cloth_in(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
{

    E1 = E2;
    E2 = E3;
    E3 = target_height - (sum_height - init_sum_height);
    float delta_height = KP * E3 + KI * (E1 + E2 + E3) + KD * ((E3 - E2) - (E2 - E1));
    delta_height = abs(delta_height) > 1.0 ? (delta_height / abs(delta_height) * 1.0) : delta_height;

    switch (flag_)
    {
    case init_position:
        if ((ros::Time::now() - init_begin) > init_duration)
            flag_ = force_control;
        init_sum_height = (init_sum_height + sum_height) / 2;
        ROS_INFO("init_position");
        break;

    case force_control:
#ifdef DISABLE_FORCE_CONTROL
        flag_ = moveR_up;
        msg_R.X_Axis = msg_R.X_Axis - deltaXGrip;
        ROS_INFO("deltaXGrip activated");
        ros::Duration(0.5).sleep();
#else
        ROS_INFO("force_control");
        /*在力控时很容易出现对不准失控的情况，所以力控过程要将手指对正，然后校正x*/
        if ((sum_height - init_sum_height) < target_height)
        {
            /*力控*/
            msg_R.X_Axis = msg_R.X_Axis - delta_height;
        }
        else
        {
            flag_ = init_Pos_after_force;
            init_begin = ros::Time::now();
        }
#endif
        break;

    case init_Pos_after_force:
        ROS_INFO("init_Pos_after_force");
        msg_L.Y_Axis = InitL_PosY;
        msg_R.Y_Axis = InitR_PosY;
        msg_L.Z_Angle = start_angle;
        if ((ros::Time::now() - init_begin) > ros::Duration(3))
            flag_ = moveR_up;
        break;

    case moveR_up:
        ROS_INFO("moveR_up");
        action5(msg_L, msg_R, deltaYDigit);
        /*Get Digit Data Here*/
        if (msg_R.Y_Axis > YDigitUpperLim)
            flag_ = moveR_down;
        break;

    case moveR_down:
        ROS_INFO("moveR_down");
        action6(msg_L, msg_R, deltaYDigit);
        if (msg_R.Y_Axis < YDigitLowerLim)
        {
            flag_ = moveR_reset;
            ros::Duration(1).sleep();
        }
        break;

    case moveR_reset:
        ROS_INFO("moveR_reset"); // Reset Digit Position
        action5(msg_L, msg_R, deltaYDigit);
        if (msg_R.Y_Axis > InitR_PosY)
        {
            flag_ = turnL_clock;
            ros::Duration(1).sleep();
        }
        break;

    case turnL_clock:
        ROS_INFO("turnL_clock, start_angle = %f", start_angle);
        ROS_INFO("Z_Angle = %f", msg_L.Z_Angle);
        action1(msg_L, msg_R, delta_theta);

        if (msg_L.Z_Angle > (theta_threshold / 2))
        {
            flag_ = moveL_up;
            ros::Duration(1).sleep();
        }
        break;

    case moveL_up:
        action4(msg_L, msg_R, delta_theta);
        // if((msg_L.Y_Axis-InitL_PosY)>((theta_threshold/2)*deg2rad*Radius))
        //     flag_=turnL_anticlock;
        if ((msg_L.Y_Axis - InitL_PosY) > delta_ythreshold)
            flag_ = turnL_anticlock;
        ROS_INFO("moveL_up");
        break;

    case turnL_anticlock:
        ROS_INFO("turnL_anticlock");
        action2(msg_L, msg_R, -delta_theta);
        if (msg_L.Z_Angle < start_angle)
        {
            flag_ = init_Pos_after_force;
            count++;
            ROS_INFO("The %d Move", count);
        }
#ifdef COUNT_MODE
        if (count >= COUNT_MODE)
        {
            flag_ = stop;
            ROS_INFO("Move times : %d", count);
        }
#endif
        break;

    case stop:
        break;

    default:
        break;
    }

    return;
}

void Move_cloth::Move_Cloth_out(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
{

    E1 = E2;
    E2 = E3;
    E3 = target_height - (sum_height - init_sum_height);
    float delta_height = KP * E3 + KI * (E1 + E2 + E3) + KD * ((E3 - E2) - (E2 - E1));
    delta_height = abs(delta_height) > 1.0 ? (delta_height / abs(delta_height) * 1.0) : delta_height;

    switch (flag_)
    {
    case init_position:
        if ((ros::Time::now() - init_begin) > init_duration)
            flag_ = force_control;
        init_sum_height = (init_sum_height + sum_height) / 2;
        break;
    case force_control:

/*在力控时很容易出现对不准失控的情况，所以力控过程要将手指对正，然后校正x*/
#ifdef Modify_digit_sensor
        if ((sum_height - init_sum_height) < target_height)
        {
            /*力控*/
            msg_R.X_Axis = msg_R.X_Axis - delta_height;
        }
        else
        {
            flag_ = init_Pos_after_force;
            init_begin = ros::Time::now();
        }
#else
        if (sum_height < target_height)
        {
            /*力控*/
            msg_R.X_Axis = msg_R.X_Axis - delta_height;
        }
        else
        {
            flag_ = init_Pos_after_force;
            init_begin = ros::Time::now();
        }
#endif

#ifdef no_force_control
        flag_ = turnL_clock;
#endif
        break;

    case init_Pos_after_force:
        msg_L.Y_Axis = InitL_PosY;
        msg_R.Y_Axis = InitR_PosY;
        msg_L.Z_Angle = start_angle;
        if ((ros::Time::now() - init_begin) > ros::Duration(3))
            flag_ = turnL_clock;
        break;

    case turnL_anticlock:
        action1(msg_L, msg_R, -delta_theta);
        if (msg_L.Z_Angle < (-theta_threshold / 2))
            flag_ = moveL_down;
        ROS_INFO("turnL_anticlock");
        break;

    case moveL_down:
        action4(msg_L, msg_R, -delta_theta);
        if ((InitL_PosY - msg_L.Y_Axis) > delta_ythreshold)
            flag_ = turnL_clock;
        ROS_INFO("moveL_down");
        break;

    case turnL_clock:
        action2(msg_L, msg_R, delta_theta);

        if (msg_L.Z_Angle > (start_angle))
        {
            flag_ = turnL_anticlock;
            count++;
            ROS_INFO("The %d Move", count);
        }
#ifdef COUNT_MODE
        if (count == 3)
        {
            flag_ = stop;
            ROS_INFO("Move times : 10");
        }
#endif

        ROS_INFO("turnL_clock");
        break;

    case stop:
        break;

    default:
        break;
    }

    return;
}

void Move_cloth::action1(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float delta_theta)
{
    /*两种情况
    1：V0_L=0,wR!=0,V0_R=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    2：V0=wR=V0_R!=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    以下采用情况1。
    */
    float delta_y = delta_theta * deg2rad * Radius;
    msg_L.Z_Angle = msg_L.Z_Angle + delta_theta;
    return;
}
void Move_cloth::action2(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float delta_theta)
{
    /*V0_L=wR!=0,V_R=0;接触点相对Digit上移，接触点处两表面相对静止*/
    float modify_delta_y = delta_theta > 0 ? Modify : -Modify;
    float delta_y = delta_theta * deg2rad * Radius + modify_delta_y; //可能因为位置精度的问题，需要增加一个修正量，来保证速度的一致
    msg_L.Z_Angle = msg_L.Z_Angle + delta_theta;
    msg_L.Y_Axis = msg_L.Y_Axis + delta_y;
    return;
}
void Move_cloth::action3(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float delta_theta)
{
    /*两种情况
    1：V0_L=0,wR!=0,V0_R=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    2：V0=wR=V0_R!=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    以下采用情况2。
    */
    float delta_y = delta_theta * deg2rad * Radius;
    msg_L.Z_Angle = msg_L.Z_Angle + delta_theta;
    msg_L.Y_Axis = msg_L.Y_Axis + delta_y;
    msg_R.Y_Axis = msg_R.Y_Axis + delta_y;
    return;
}
void Move_cloth::action4(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float delta_theta)
{
    /*wR=0,V0_L!=0,V0_R=0;接触点相对于digit上移*/
    float delta_y = delta_theta * deg2rad * Radius;
    msg_L.Y_Axis = msg_L.Y_Axis + delta_y;
    return;
}
void Move_cloth::action5(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float delta_digit)
{
    /*digit上移*/
    msg_R.Y_Axis = msg_R.Y_Axis + delta_digit;
    return;
}
void Move_cloth::action6(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float delta_digit)
{
    /*digit下移*/
    msg_R.Y_Axis = msg_R.Y_Axis - delta_digit;
    return;
}

string Move_cloth::get_HdControl_state()
{
    // enum flag{
    //      init_position=0,
    //      force_control,
    //      init_Pos_after_force,
    //      turnL_clock/*通过摩擦面带动布料向手内*/,
    //      moveL_up,/*moveL_up与turnL_anticlock配合，完成接触区域的转换*/
    //      moveL_down,
    //      turnL_anticlock,/**/
    //      stop
    //  };
    //  flag flag_;
    string result;
    switch (flag_)
    {
    case init_position:
        result = "init_position";
        break;
    case force_control:
        result = "force_control";
        break;
    case init_Pos_after_force:
        result = "init_Pos_after_force";
        break;
    case turnL_clock:
        result = "turnL_clock";
        break;
    case moveL_up:
        result = "moveL_up";
        break;
    case moveL_down:
        result = "moveL_down";
        break;
    case turnL_anticlock:
        result = "turnL_anticlock";
        break;
    case moveR_up:
        result = "moveR_up";
        break;
    case moveR_down:
        result = "moveR_down";
        break;
    case moveR_reset:
        result = "moveR_reset";
        break;
    case stop:
        result = "stop";
        break;
    default:
        break;
    }
    return result;
}
