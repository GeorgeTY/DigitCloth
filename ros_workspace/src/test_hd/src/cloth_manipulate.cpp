#include "cloth_manipulate.h"

using namespace std;

vector<float> Calculate_SubHeight(const get_height::Height &current_height)
{
    vector<float> SubHeight(6, 0.0); // 将传感器数据初步分为6个区数据,传感器数据为6*3
    SubHeight[0] = current_height.height_data0[0] + current_height.height_data0[1] + current_height.height_data0[2];
    SubHeight[1] = current_height.height_data1[0] + current_height.height_data1[1] + current_height.height_data1[2];
    SubHeight[2] = current_height.height_data2[0] + current_height.height_data2[1] + current_height.height_data2[2];
    SubHeight[3] = current_height.height_data3[0] + current_height.height_data3[1] + current_height.height_data3[2];
    SubHeight[4] = current_height.height_data4[0] + current_height.height_data4[1] + current_height.height_data4[2];
    SubHeight[5] = current_height.height_data5[0] + current_height.height_data5[1] + current_height.height_data5[2];
    // ROS_INFO("Sub0: %lf", SubHeight[0]);
    // ROS_INFO("Sub1: %lf", SubHeight[1]);
    // ROS_INFO("Sub2: %lf", SubHeight[2]);
    // ROS_INFO("Sub3: %lf", SubHeight[3]);
    // ROS_INFO("Sub4: %lf", SubHeight[4]);
    // ROS_INFO("Sub5: %lf", SubHeight[5]);

    return SubHeight;
}
void Move_cloth::Init(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
{
    flag_ = init_position;
    init_begin = ros::Time::now();
    init_duration = ros::Duration(6);

    deta_theta = w * (1 / ((float)frequency));
    deta_ythreshold = ((theta_threshold / 2) * deg2rad * Radius) + (theta_threshold / 2 / deta_theta) * Modify; // 默认是对中位置开始
    start_angle = 0;

    E1 = 0;
    E2 = 0;
    E3 = 0;
    init_sum_height = 0.0;

    count = 0;

    msg_L.X_Axis = InitL_PosX;
    msg_L.Y_Axis = 135.0 /*132.0*/;
    msg_L.Z_Angle = InitL_PosAngle;
    msg_R.X_Axis = 0;
    msg_R.Y_Axis = 130.0;
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
    float deta = (theta_threshold / 2 - start_angle);
    deta_ythreshold = (deta * deg2rad * Radius) + (deta / deta_theta) * Modify;

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
    float deta = (theta_threshold / 2 + start_angle);
    deta_ythreshold = (deta * deg2rad * Radius) + (deta / deta_theta) * Modify;

    init_begin = ros::Time::now();

    return;
}

void Move_cloth::Move_Cloth_in(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, get_height::Height &current_height)
{

    /*digit力反馈情况*/
    vector<float> sub_height = Calculate_SubHeight(current_height);
    sum_height = accumulate(sub_height.begin(), sub_height.end(), 0.0);
    upArea_height = accumulate(sub_height.begin(), sub_height.begin() + 4, 0.0);
    downArea_height = accumulate(sub_height.begin() + 4, sub_height.end(), 0.0);
    sum_height = abs(sum_height);
    ROS_INFO("Sum_force: %lf", sum_height);
    // ROS_INFO("Up_force: %lf",upArea_height);
    // ROS_INFO("Down_force: %lf",downArea_height);

    E1 = E2;
    E2 = E3;
    E3 = target_height - (sum_height - init_sum_height);
    float deta_height = KP * E3 + KI * (E1 + E2 + E3) + KD * ((E3 - E2) - (E2 - E1));
    deta_height = abs(deta_height) > 1.0 ? (deta_height / abs(deta_height) * 1.0) : deta_height;

    switch (flag_)
    {
    case init_position:
        if ((ros::Time::now() - init_begin) > init_duration)
            flag_ = force_control;
        init_sum_height = (init_sum_height + sum_height) / 2;
        ROS_INFO("init_position");
        break;
    case force_control:
        ROS_INFO("force_control");
        /*在力控时很容易出现对不准失控的情况，所以力控过程要将手指对正，然后校正x*/
        if ((sum_height - init_sum_height) < target_height)
        {
            /*力控*/
            msg_R.X_Axis = msg_R.X_Axis - deta_height / 10;
        }
        else
        {
            flag_ = init_Pos_after_force;
            init_begin = ros::Time::now();
        }
#ifdef no_force_control
        flag_ = turnL_clock;
#endif
        break;

    case init_Pos_after_force:
        ROS_INFO("init_Pos_after_force");
        msg_L.Y_Axis = InitL_PosY;
        msg_R.Y_Axis = InitR_PosY;
        msg_L.Z_Angle = start_angle;
        if ((ros::Time::now() - init_begin) > ros::Duration(6))
            flag_ = turnL_clock;
        break;
    case turnL_clock:
        ROS_INFO("turnL_clock");
        action1(msg_L, msg_R, deta_theta);

        if (msg_L.Z_Angle > (theta_threshold / 2))
            flag_ = moveL_up;

        break;
    case moveL_up:

        action4(msg_L, msg_R, deta_theta);
        // if((msg_L.Y_Axis-InitL_PosY)>((theta_threshold/2)*deg2rad*Radius))
        //     flag_=turnL_anticlock;
        if ((msg_L.Y_Axis - InitL_PosY) > deta_ythreshold)
            flag_ = turnL_anticlock;
        ROS_INFO("moveL_up");
        break;

    case turnL_anticlock:
        action2(msg_L, msg_R, -deta_theta);
        if (msg_L.Z_Angle < start_angle)
        {
            flag_ = turnL_clock;
            count++;
            // ROS_INFO("The %d Move",count);
        }
#ifdef COUNT_MODE
        if (count == COUNT_MODE)
        {
            flag_ = stop;
            ROS_INFO("Move times : 10");
        }
#endif

        ROS_INFO("turnL_anticlock");
        break;
    case stop:
        break;

    default:
        break;
    }

    return;
}

void Move_cloth::Move_Cloth_out(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, get_height::Height &current_height)
{

    /*digit力反馈情况*/
    vector<float> sub_height = Calculate_SubHeight(current_height);
    sum_height = accumulate(sub_height.begin(), sub_height.end(), 0.0);
    upArea_height = accumulate(sub_height.begin(), sub_height.begin() + 4, 0.0);
    downArea_height = accumulate(sub_height.begin() + 4, sub_height.end(), 0.0);
    ROS_INFO("Sum_force: %lf", sum_height);
    // ROS_INFO("Up_force: %lf",upArea_height);
    // ROS_INFO("Down_force: %lf",downArea_height);

    E1 = E2;
    E2 = E3;
    E3 = target_height - (sum_height - init_sum_height);
    float deta_height = KP * E3 + KI * (E1 + E2 + E3) + KD * ((E3 - E2) - (E2 - E1));
    deta_height = abs(deta_height) > 1.0 ? (deta_height / abs(deta_height) * 1.0) : deta_height;

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
            msg_R.X_Axis = msg_R.X_Axis - deta_height;
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
            msg_R.X_Axis = msg_R.X_Axis - deta_height / 10;
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
        if ((ros::Time::now() - init_begin) > ros::Duration(6))
            flag_ = turnL_clock;
        break;

    case turnL_anticlock:
        action1(msg_L, msg_R, -deta_theta);
        if (msg_L.Z_Angle < (-theta_threshold / 2))
            flag_ = moveL_down;
        ROS_INFO("turnL_anticlock");
        break;

    case moveL_down:
        action4(msg_L, msg_R, -deta_theta);
        if ((InitL_PosY - msg_L.Y_Axis) > deta_ythreshold)
            flag_ = turnL_clock;
        ROS_INFO("moveL_down");
        break;

    case turnL_clock:
        action2(msg_L, msg_R, deta_theta);

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

void Move_cloth::action1(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta)
{
    /*两种情况
    1：V0_L=0,wR!=0,V0_R=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    2：V0=wR=V0_R!=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    以下采用情况1。
    */
    float deta_y = deta_theta * deg2rad * Radius;
    msg_L.Z_Angle = msg_L.Z_Angle + deta_theta;
    return;
}
void Move_cloth::action2(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta)
{
    /*V0_L=wR!=0,V_R=0;接触点相对Digit上移，接触点处两表面相对静止*/
    float modify_deta_y = deta_theta > 0 ? Modify : -Modify;
    float deta_y = deta_theta * deg2rad * Radius + modify_deta_y; // 可能因为位置精度的问题，需要增加一个修正量，来保证速度的一致
    msg_L.Z_Angle = msg_L.Z_Angle + deta_theta;
    msg_L.Y_Axis = msg_L.Y_Axis + deta_y;
    return;
}
void Move_cloth::action3(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta)
{
    /*两种情况
    1：V0_L=0,wR!=0,V0_R=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    2：V0=wR=V0_R!=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    以下采用情况2。
    */
    float deta_y = deta_theta * deg2rad * Radius;
    msg_L.Z_Angle = msg_L.Z_Angle + deta_theta;
    msg_L.Y_Axis = msg_L.Y_Axis + deta_y;
    msg_R.Y_Axis = msg_R.Y_Axis + deta_y;
    return;
}
void Move_cloth::action4(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta)
{
    /*wR=0,V0_L!=0,V0_R=0;接触点相对于digit上移*/
    float deta_y = deta_theta * deg2rad * Radius;
    msg_L.Y_Axis = msg_L.Y_Axis + deta_y;
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
    case stop:
        result = "stop";
        break;
    default:
        break;
    }
    return result;
}
