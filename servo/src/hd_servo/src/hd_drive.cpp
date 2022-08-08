#include<ros/ros.h>
#include"hd_drive.h"

hd_drive::hd_drive(int finger=Finger_L)
{
    /*for test*/
    cout<<"Constructor is called !"<<endl;

    /*进行坐标变换，左右手对称*/
    if(finger==Finger_L){
        flag_=0;
    }
    else{
        flag_=1;
    }

    char * serial="/dev/ttyUSB0";//串行端口号
    if(!sms_.begin(115200,serial)){
        cout<<"Failed to init sms motor !"<<endl;
       
    }
    else{
        cout<<"sms motors init succeed !"<<endl;
    }

    // Goal_Pos=MatD31(0.0,0.0,0.0);
    // Origin_Pos=MatD31(0.0,0.0,0.0);

}
void hd_drive::Finger_init()
{
    /*中位校准，使能扭矩*/
    if(flag_==Finger_L){
        for(int i=0;i<3;++i){

            #ifdef pos_calibration
                sms_.CalibrationOfs(ID_L_[i]);
            #endif

            sms_.EnableTorque(ID_L_[i],1);//1:使能扭矩；0：释放扭矩
        }
        cout<<"Finger_L init succeed !!!"<<endl;
    }else{
        for(int i=0;i<3;++i){

            #ifdef pos_calibration
                sms_.CalibrationOfs(ID_R_[i]);
            #endif

            sms_.EnableTorque(ID_R_[i],1);
        }
        cout<<"Finger_R init succeed !!!"<<endl;
    }

    /*初始化起始末端位姿*/
    Motor_feedback();//获取当前电机关节角
    Origin_Pos=Forward_kinematics(Joint_theta,flag_);//运动学正解求末端位姿
    Goal_Pos=Origin_Pos; 

    /*for test*/
    std::cout<<"Origin Motor_Pos:*-*"<<Motor_Pos[0]<<"*-*"<<Motor_Pos[1]<<"*-*"<<Motor_Pos[2]<<endl;
    std::cout<<"Origin Joint_theta:*-*"<<Joint_theta[0]<<"*-*"<<Joint_theta[1]<<"*-*"<<Joint_theta[2]<<endl;
    std::cout<<"Origin pos:*-*"<<Origin_Pos(1,1) <<"*-*"<<Origin_Pos(2,1)<<"*-*"<<Origin_Pos(3,1)<<endl;
    
    
    

}
hd_drive::~hd_drive()
{    
    /*for test */
    //cout<<"Destructor is called !"<<endl;

}

void hd_drive::Finger_end()
{
    /*舵机回位，关闭扭矩功能*/
    if(flag_==Finger_L){
        for(int i=0;i<3;++i){
            sms_.WritePosEx(ID_L_[i],2300,200);
            //sms_.EnableTorque(ID_L_[i],0);//1:使能扭矩；0：释放扭矩****如果关节回位还没有完成就释放扭矩使能会导致异常
        }
        cout<<"Finger_L return homePos !!!"<<endl;
    }else{
        for(int i=0;i<3;++i){
            sms_.WritePosEx(ID_R_[i],2300,200);
            //sms_.EnableTorque(ID_R_[i],0);
        }
        cout<<"Finger_R return homePos !!!"<<endl;
    }

    //sms_.end();//一个手指调用关闭驱动后，会导致另一个手指无法回位
}

void hd_drive::Cmd_publish(MATRIX_D Cmdjoints_theta)
{
    // /*for test*/
    // cout<<"*--theta--"<<Cmdjoints_theta(1,1)<<" "<<Cmdjoints_theta(2,1)<<" "<<Cmdjoints_theta(3,1)<<endl;
   
    /*θ角转换为舵机指令*/
    Cmdjoints_theta(1,1)=theta_1_calibration-Cmdjoints_theta(1,1);
    Cmdjoints_theta(2,1)=theta_2_calibration-Cmdjoints_theta(2,1);
    Cmdjoints_theta(3,1)=theta_3_calibration-Cmdjoints_theta(3,1);

    // /*for test*/
    // cout<<"*--theta--"<<Cmdjoints_theta(1,1)<<" "<<Cmdjoints_theta(2,1)<<" "<<Cmdjoints_theta(3,1)<<endl;


    for(int i=1;i<=3;++i){
        Cmdjoints_theta(i,1)=Cmdjoints_theta(i,1)*DegtoStep;//舵机角度转换为步数指令
    }

    /*将指令转换成电机接口允许的类型*/
    s16 Cmd_temp[3];
    u16 Speed_temp[3];
    for(int i=0;i<3;++i){
        Cmd_temp[i]=(s16)(Cmdjoints_theta(i+1,1));
        Speed_temp[i]=(u16)(Speed_[i]*DegtoStep);
        if(Cmd_temp[i]>MotorCmd_LimtLow_[i]&&Cmd_temp[i]<MotorCmd_LimtHig_[i]){
            continue;
        }else{
            std::cout<<"Motor Cmd Out Of Limit --- Shutdown RosNode!!!"<<endl;
            ros::shutdown();
            
        }
    }
    
    if(flag_==0){
        sms_.SyncWritePosEx(ID_L_,IDN_,Cmd_temp,Speed_temp,ACC_);
    }
    else{
        sms_.SyncWritePosEx(ID_R_,IDN_,Cmd_temp,Speed_temp,ACC_);
    }

    /*for test*/
    // cout<<"*--theta--"<<Cmdjoints_theta(1,1)<<" "<<Cmdjoints_theta(2,1)<<" "<<Cmdjoints_theta(3,1)<<endl;
    // cout<<"--motorPos--"<<Cmd_temp[0]<<" "<<Cmd_temp[1]<<" "<<Cmd_temp[2]<<endl;
}

void hd_drive::Motor_feedback()
{
    if(flag_==Finger_L){
        for(int i=0;i<3;++i){
            // Motor_Load[i]=sms_.ReadLoad(ID_L_[i]);
            // Motor_Speed[i]=sms_.ReadSpeed(ID_L_[i]);
            // Motor_Temper[i]=sms_.ReadTemper(ID_L_[i]);
            Motor_Pos[i]=sms_.ReadPos(ID_L_[i]);
            // Motor_Voltage[i]=sms_.ReadVoltage(ID_L_[i]);
            // Motor_Current[i]=sms_.ReadCurrent(ID_L_[i]);
        }
    }else{
        for(int i=0;i<3;++i){
            // Motor_Load[i]=sms_.ReadLoad(ID_R_[i]);
            // Motor_Speed[i]=sms_.ReadSpeed(ID_R_[i]);
            // Motor_Temper[i]=sms_.ReadTemper(ID_R_[i]);
            Motor_Pos[i]=sms_.ReadPos(ID_R_[i]);
            // Motor_Voltage[i]=sms_.ReadVoltage(ID_R_[i]);
            // Motor_Current[i]=sms_.ReadCurrent(ID_R_[i]);
        }
    }
    for(int i=0;i<3;++i){
        Joint_theta[i]=Motor_Pos[i]*SteptoDeg;
    }
    Joint_theta[0]=-(Joint_theta[0]-270.0);
    Joint_theta[1]=-(Joint_theta[1]-180.0);
    Joint_theta[2]=-(Joint_theta[2]-180.0);
}
