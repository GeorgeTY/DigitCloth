#include "auto_clothmanipulation.h"
using namespace std;

#define DIS_MODIFY 8 // 修正，因为检测到光流位移比较大时，其实已经滑动了一段距离，所以，以此值修正 

vector<float> calculateSubHeight(const get_height::Height &current_height){
    vector<float> SubHeight(6,0.0);         //将传感器数据初步分为6个区数据,传感器数据为6*3
    SubHeight[0]=current_height.height_data0[0]+current_height.height_data0[1]+current_height.height_data0[2];
    SubHeight[1]=current_height.height_data1[0]+current_height.height_data1[1]+current_height.height_data1[2];
    SubHeight[2]=current_height.height_data2[0]+current_height.height_data2[1]+current_height.height_data2[2];
    SubHeight[3]=current_height.height_data3[0]+current_height.height_data3[1]+current_height.height_data3[2];
    SubHeight[4]=current_height.height_data4[0]+current_height.height_data4[1]+current_height.height_data4[2];
    SubHeight[5]=current_height.height_data5[0]+current_height.height_data5[1]+current_height.height_data5[2];
    // ROS_INFO("Sub0: %lf",SubHeight[0]);
    // ROS_INFO("Sub1: %lf",SubHeight[1]);
    // ROS_INFO("Sub2: %lf",SubHeight[2]);
    // ROS_INFO("Sub3: %lf",SubHeight[3]);
    // ROS_INFO("Sub4: %lf",SubHeight[4]);
    // ROS_INFO("Sub5: %lf",SubHeight[5]);
    return SubHeight;
}

// /*计算角度的有点问题，当移动距离为2时，给的起始角移动不了织物，需要修改一下*/
// float Move_cloth::getAnglefromDis(int nextUnit_dis){
//     //1)根据nextUnit_dis计算动作单元初始角
//     if (nextUnit_dis == 0) {
//         return -100.0; // 当收到目标移动距离为0时，将夹爪居中停止
//     }
//     if (nextUnit_dis < 0) {
//         return -200.0;
//     }
//     nextUnit_dis = nextUnit_dis > 0 ? nextUnit_dis : -nextUnit_dis; // 取绝对值，即只能单方向运动
//     if (nextUnit_dis > Max_dis) {
//         // 超出单元最大移动距离
//         return Rstart_threahold;
//     }
//     float mid_dis = Max_dis / 2.0;
//     if (nextUnit_dis > mid_dis) {
//         return (Rstart_threahold - Rmid_threshold)*(nextUnit_dis - mid_dis)/mid_dis   + Rmid_threshold;
//     } else {
//         return (Rmid_threshold - Rend_threshold)*(nextUnit_dis)/mid_dis + Rend_threshold; 
//     }
// }

/*计算角度的有点问题，当移动距离为2时，给的起始角移动不了织物，需要修改一下*/
/*经过测试，结束转动动作前，布料开始滑动的角度是15度左右，根据这个角度修正起始角计算公式*/
float Move_cloth::getAnglefromDis(int nextUnit_dis){
    //1)根据nextUnit_dis计算动作单元初始角
    if (nextUnit_dis == 0) {
        return -100.0; // 当收到目标移动距离为0时，将夹爪居中停止
    }
    if (nextUnit_dis < 0) {
        return -200.0;
    }
    float Rend = 15.0;
    float max_dis = (Rend - Rstart_threahold)*deg2rad*Radius;
    if (nextUnit_dis > max_dis) {
        // 超出单元最大移动距离
        return Rstart_threahold;
    }
    return (Rstart_threahold - Rend)*(nextUnit_dis)/max_dis + Rend;    
}

void Move_cloth::Init(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R){
    flag_=Init_position;
    init_begin=ros::Time::now();
    init_duration=ros::Duration(6);
    detectDis_pub = node.advertise<std_msgs::Int32>("Cloth_Detected_Distance",1);

    deta_theta=w*(1/((float)frequency));
    deta_ythreshold=((theta_threshold/2)*deg2rad*Radius)+(theta_threshold/2/deta_theta)*Modify;//默认是对中位置开始
    start_angle=0;

    E1=0;
    E2=0;
    E3=0;
    init_sum_height=0.0;

    angle_turn = 0.0;
    Turn_startangel = 0.0;
    flag_pub = true;

    count=0;

    msg_L.X_Axis = InitL_PosX; msg_L.Y_Axis = 135.0/*132.0*/; msg_L.Z_Angle = InitL_PosAngle;
    msg_R.X_Axis = InitR_PosX; msg_R.Y_Axis = 135.0; msg_R.Z_Angle = InitR_PosAngle;

    return;
}


void Move_cloth::MoveClothin(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R,get_height::Height &current_height,int nextUnit_dis,float optical_flow){
        
        // int nextUnit_dis 下一操作单元的目标移动距离，在力控之后使用，和在每次的slid_action后使用
        // nextUnit_dis 应该再0-9之间，
        //策略1：直接按理想状态去算其角度，然后控制运动，最后通过反馈计算实际移动距离，然后修正下一阶段目标距离，该方法实现相对应该简单
                // 但是对于小距离的移动目标，如果不加修正的直接计算实际移动距离，那么可能会导致设置小距离移动目标，但是根本对织物起不到移动效果
        //策略2：根据反馈与下一阶段目标移动距离进行修正，修正的逻辑需要仔细写一下
    
        /*digit力反馈情况*/
        vector<float> sub_height=calculateSubHeight(current_height);
        sum_height=accumulate(sub_height.begin(),sub_height.end(),0.0);
        upArea_height=accumulate(sub_height.begin(),sub_height.begin()+4,0.0);
        downArea_height=accumulate(sub_height.begin()+4,sub_height.end(),0.0);
        // ROS_INFO("Sum_force: %lf",sum_height);
        // ROS_INFO("Up_force: %lf",upArea_height);
        // ROS_INFO("Down_force: %lf",downArea_height);
        
        E1=E2;
        E2=E3;
        E3=target_height-(sum_height-init_sum_height);
        float deta_height=KP*E3+KI*(E1+E2+E3)+KD*((E3-E2)-(E2-E1));
        deta_height=abs(deta_height)>1.0?(deta_height/abs(deta_height)*1.0):deta_height;
 

    switch (flag_)
    {
        case Init_position:
            if((ros::Time::now()-init_begin)>init_duration)
                flag_=Force_control;
            init_sum_height=(init_sum_height+sum_height)/2;
            ROS_INFO("Init_position");
            break;
        case Force_control:
            ROS_INFO("Force_control");
            /*在力控时很容易出现对不准失控的情况，所以力控过程要将手指对正，然后校正x*/
            if((sum_height-init_sum_height)<target_height){
                /*力控*/
                msg_R.X_Axis=msg_R.X_Axis-deta_height;
            }else{
                flag_=Roll_action;
                Turn_startangel = getAnglefromDis(nextUnit_dis);
                if (Turn_startangel < -99.0 && Turn_startangel > -199.0) {
                    flag_ = Stop;
                    Turn_startangel = 0;
                } 
                if (Turn_startangel < -199.0) {
                    flag_ = Release;
                }
                init_begin=ros::Time::now();
            }
            #ifdef no_force_control
            
                flag_=Roll_action;
                Turn_startangel = getAnglefromDis(nextUnit_dis);
                if (Turn_startangel < -99.0 && Turn_startangel > -199.0) {
                    flag_ = Stop;
                    Turn_startangel = 0;
                } 
                if (Turn_startangel < -199.0) {
                    flag_ = Release;
                }
            #endif
            break;
        case Turn_action:
            ROS_INFO("Turn_action");
            // 滚动角度,和旋转角度是相等的
            
            if (optical_flow > 2.0 && flag_pub && msg_L.Z_Angle > 0.0) {
                // ros::shutdown();
                flag_pub = false;
                float tmp = msg_L.Z_Angle;

                // ROS_INFO("Start_flow:%lf",tmp-DIS_MODIFY);

                tmp = tmp - DIS_MODIFY - Turn_startangel;
                detect_dis.data = min(min((int)(angle_turn*deg2rad*Radius), (int)(tmp*deg2rad*Radius)),Max_dis);
                detectDis_pub.publish(detect_dis);
                // ROS_INFO("calc_theta:%lf", tmp);
                // ROS_INFO("angle_turn:%lf",angle_turn);
                
                // ROS_INFO("StartAngle_Turn:%lf",Turn_startangel);
            }
            if(msg_L.Z_Angle > Rend_angle) {
                msg_L.Z_Angle = Rend_angle; // 防止误差累计,每循环修正一次
                flag_ = Slid_action;
                if (flag_pub) {
                    flag_pub = false;
                    detect_dis.data = min(min((int)(angle_turn*deg2rad*Radius), nextUnit_dis),Max_dis);
                    detectDis_pub.publish(detect_dis);
                }
            }

            actionTurn(msg_L,msg_R,deta_theta);
            angle_turn += deta_theta; // 记录旋转的角度，与滑移的角度相等
            break;
        case Slid_action:
            ROS_INFO("Slid_action");
            actionSlid(msg_L,msg_R,deta_theta);
            angle_turn -= deta_theta;
            if (angle_turn < 0) {
                flag_pub = true;
                angle_turn = 0;
                flag_ = Roll_action;
                msg_L.Y_Axis = InitL_PosY + Rend_angle*deg2rad*Radius;
                Turn_startangel = getAnglefromDis(nextUnit_dis);
                if (Turn_startangel < -99.0 && Turn_startangel > -199.0) {
                    flag_ = Stop;
                    Turn_startangel = 0;
                } 
                if (Turn_startangel < -199.0) {
                    flag_ = Release;
                }
            }             
            break;
        
        case Roll_action:
            ROS_INFO("Roll_action");
            actionRoll(msg_L, msg_R, deta_theta);
            if (msg_L.Z_Angle < Turn_startangel) {
                msg_L.Z_Angle = Turn_startangel;
                flag_ = Turn_action;
            }
            break;
        case Stop:
            ROS_INFO("Stop");
            // 停止夹爪的过程
            actionRoll(msg_L, msg_R, deta_theta);
            if (msg_L.Z_Angle < Turn_startangel) {
                msg_L.Z_Angle = Turn_startangel;
                flag_ = Rest;
            }
            break;
        case Rest:
            ROS_INFO("Rest");
            // 停止夹爪，等待后续指令
            Turn_startangel = getAnglefromDis(nextUnit_dis);
            if (Turn_startangel > -99.0) {
                flag_ = Roll_action;
            }
            if (Turn_startangel < -199.0) {
                flag_ = Release;
            }
            break;
        case Release:
            ROS_INFO("Release");
            msg_L.X_Axis = -70; msg_L.Y_Axis = 135; msg_L.Z_Angle = -10;
            msg_R.X_Axis = 40; msg_R.Y_Axis = 135; msg_R.Z_Angle = -10;
            flag_ = Final;
            break;
        default:
            ros::shutdown();
            break;
        
    }

    return ;
}

void Move_cloth::actionTurn(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta) {
    float deta_z = deta_theta;
    msg_L.Z_Angle = msg_L.Z_Angle + deta_z;
}

void Move_cloth::actionSlid(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta) {
    float deta_y = deta_theta * deg2rad * Radius;
    msg_L.Y_Axis = msg_L.Y_Axis + deta_y;
}

void Move_cloth::actionRoll(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta) {
    float deta_z = deta_theta;
    float deta_y = deta_theta * deg2rad * Radius;
    msg_L.Y_Axis = msg_L.Y_Axis - deta_y;
    msg_L.Z_Angle = msg_L.Z_Angle - deta_z;
}

string Move_cloth::getHdControlstate(){
        //enum flag{
        //     Init_position=0,
        //     Force_control,
        //     InitPos_AfterForce,
        //     Turn_action, // 第一元动作，初始位置应该根据目标移动距离来计算
        //     Slid_action, 
        //     Roll_action, 
        //     Stop, 
        // };
        // flag flag_;
        string result;
        switch(flag_){
            case Init_position:
                result="Init_position";
                break;
            case Force_control:
                result="Force_control";
                break;
            case InitPos_AfterForce:
                result="InitPos_AfterForce";
                break;
            case Turn_action:
                result="Turn_action";
                break;
            case Slid_action:
                result="Slid_action";
                break;
            case Roll_action:
                result="Roll_action";
                break;
            case Stop:
                result="Stop";
                break;  
            case Rest:
                result = "Rest";
            case Release:
                result = "Release";
            case Final:
                result = "Final";
            default:
                break;
        }
        return result;
}


