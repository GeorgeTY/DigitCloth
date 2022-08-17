#include "kukaservo_lib.h"

using namespace std;

Servo_Lib::Servo_Lib(){
    leftclient=nh.serviceClient<kukafri_hw::setMoveMode>("/left/setMoveMode");
    lefthomeclient=nh.serviceClient<kukafri_hw::moveToHome>("/left/MoveToHome");
    LeftEulerXYZ=nh.advertise<kukafri_hw::kukaCmdPosE>("/left/cmdPos_EulerZYX",1);
    LeftQuaternion=nh.advertise<kukafri_hw::kukaCmdPosQ>("/left/cmdPos_Quaternion",1);
    LeftJoint=nh.advertise<kukafri_hw::kukaCmdJoint>("/left/cmdJoint",1);

    rightclient=nh.serviceClient<kukafri_hw::setMoveMode>("/right/setMoveMode");
    righthomeclient=nh.serviceClient<kukafri_hw::moveToHome>("/right/MoveToHome");
    RightEulerXYZ=nh.advertise<kukafri_hw::kukaCmdPosE>("/right/cmdPos_EulerZYX",1);
    RightQuaternion=nh.advertise<kukafri_hw::kukaCmdPosQ>("/right/cmdPos_Quaternion",1);
    RightJoint=nh.advertise<kukafri_hw::kukaCmdJoint>("/right/cmdJoint",1);

    LeftKukaPos=nh.subscribe("/left/curState",1,&Servo_Lib::leftkukaCb,this);
    RightKukaPos=nh.subscribe("/right/curState",1,&Servo_Lib::rightkukaCb,this);

    homesrv.request.no_use=0;
    SetLeftMoveMode();
    SetRightMoveMode();
    ros::Duration(3).sleep();
}

Servo_Lib::~Servo_Lib()
{}

void Servo_Lib::leftkukaCb(const kukafri_hw::kukaState::ConstPtr& msg){
    LeftPos=msg->Pos;
    LeftEuler=msg->EulerZYX;
    leftjoint=msg->Joints;
}

void Servo_Lib::rightkukaCb(const kukafri_hw::kukaState::ConstPtr& msg){
    RightPos=msg->Pos;
    RightEuler=msg->EulerZYX;
    rightjoint=msg->Joints;
}


void Servo_Lib::SetLeftMoveMode(int moveMode,int pathMode,double moveDuration){
    kukafri_hw::setMoveMode leftsrv;
    leftsrv.request.moveMode=moveMode;
    leftsrv.request.pathMode=pathMode;
    leftsrv.request.moveDuration=moveDuration;
    leftclient.call(leftsrv);
}

void Servo_Lib::SetRightMoveMode(int moveMode,int pathMode,double moveDuration){
    kukafri_hw::setMoveMode rightsrv;
    rightsrv.request.moveMode=moveMode;
    rightsrv.request.pathMode=pathMode;
    rightsrv.request.moveDuration=moveDuration;
    rightclient.call(rightsrv);
}

/****************************************左臂相关函数****************************************************/
void Servo_Lib::MoveLeftToHome(double time){
    SetLeftMoveMode(0,0,time);
    lefthomeclient.call(homesrv);
    ros::Duration(time+0.5).sleep();

}

void Servo_Lib::MoveLeftEulerXYZ(double X_Axis,double Y_Axis,double Z_Axis,double alpha,double beta,double gamma,double time,int path){
    SetLeftMoveMode(1,path,time);
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=X_Axis;
    msg.Y_Axis=Y_Axis;
    msg.Z_Axis=Z_Axis;
    msg.alpha=alpha;
    msg.beta=beta;
    msg.gamma=gamma;
    LeftEulerXYZ.publish(msg);
    ros::Duration(time+0.5).sleep();
}

void Servo_Lib::MoveLeftQuaternion(double X_Axis,double Y_Axis,double Z_Axis,double x,double y,double z,double w,double time,int path){
    SetLeftMoveMode(1,path,time);
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=X_Axis;
    msg.Y_Axis=Y_Axis;
    msg.Z_Axis=Z_Axis;
    msg.x=x;
    msg.y=y;
    msg.z=z;
    msg.w=w;
    LeftQuaternion.publish(msg);
    ros::Duration(time+0.5).sleep();

}

void Servo_Lib::MoveLeftJoint(double Joint1,double Joint2,double Joint3,double Joint4,double Joint5,double Joint6,double Joint7,double time,int path){
    SetLeftMoveMode(0,path,time);
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=Joint1;
    msg.joint2=Joint2;
    msg.joint3=Joint3;
    msg.joint4=Joint4;
    msg.joint5=Joint5;
    msg.joint6=Joint6;
    msg.joint7=Joint7;
    LeftJoint.publish(msg);
    ros::Duration(time+0.5).sleep();

}

void Servo_Lib::MoveDLeftEulerXYZ(double dx,double dy,double dz,double dalpha,double dbeta,double dgamma,double time,int path){
    SetLeftMoveMode(1,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=LeftPos[0]+dx;
    msg.Y_Axis=LeftPos[1]+dy;
    msg.Z_Axis=LeftPos[2]+dz;
    msg.alpha=LeftEuler[0]+dalpha;
    msg.beta=LeftEuler[1]+dbeta;
    msg.gamma=LeftEuler[2]+dgamma;
    LeftEulerXYZ.publish(msg);
    ros::Duration(time+0.5).sleep();

}

void Servo_Lib::MoveDLeftQuaternion(double dX,double dY,double dZ,double dx,double dy,double dz,double dw,double time,int path){
    SetLeftMoveMode(1,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=LeftPos[0]+dX;
    msg.Y_Axis=LeftPos[1]+dY;
    msg.Z_Axis=LeftPos[2]+dZ;
    msg.x=LeftPos[3]+dx;
    msg.y=LeftPos[4]+dy;
    msg.z=LeftPos[5]+dz;
    msg.w=LeftPos[6]+dw;
    LeftQuaternion.publish(msg);
    ros::Duration(time+0.5).sleep();

}

void Servo_Lib::MoveDLeftJoint(double dJoint1,double dJoint2,double dJoint3,double dJoint4,double dJoint5,double dJoint6,double dJoint7,double time,int path){
    SetLeftMoveMode(0,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=leftjoint[0]+dJoint1;
    msg.joint2=leftjoint[1]+dJoint2;
    msg.joint3=leftjoint[2]+dJoint3;
    msg.joint4=leftjoint[3]+dJoint4;
    msg.joint5=leftjoint[4]+dJoint5;
    msg.joint6=leftjoint[5]+dJoint6;
    msg.joint7=leftjoint[6]+dJoint6;
    LeftJoint.publish(msg);
    ros::Duration(time+0.5).sleep();

}

/******************************************************************************************************/

/****************************************右臂相关函数****************************************************/
void Servo_Lib::MoveRightToHome(double time){
    SetRightMoveMode(0,0,time);
    righthomeclient.call(homesrv);
}

void Servo_Lib::MoveRightEulerXYZ(double X_Axis,double Y_Axis,double Z_Axis,double alpha,double beta,double gamma,double time,int path){
    SetRightMoveMode(1,path,time);
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=X_Axis;
    msg.Y_Axis=Y_Axis;
    msg.Z_Axis=Z_Axis;
    msg.alpha=alpha;
    msg.beta=beta;
    msg.gamma=gamma;
    RightEulerXYZ.publish(msg);
}
void Servo_Lib::MoveRightEulerXYZ(int flag, double X_Axis,double Y_Axis,double Z_Axis,double alpha,double beta,double gamma){
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=X_Axis;
    msg.Y_Axis=Y_Axis;
    msg.Z_Axis=Z_Axis;
    msg.alpha=alpha;
    msg.beta=beta;
    msg.gamma=gamma;
    RightEulerXYZ.publish(msg);
}


void Servo_Lib::MoveRightQuaternion(double X_Axis,double Y_Axis,double Z_Axis,double x,double y,double z,double w,double time,int path){
    SetRightMoveMode(1,path,time);
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=X_Axis;
    msg.Y_Axis=Y_Axis;
    msg.Z_Axis=Z_Axis;
    msg.x=x;
    msg.y=y;
    msg.z=z;
    msg.w=w;
    RightQuaternion.publish(msg);
}

void Servo_Lib::MoveRightJoint(double Joint1,double Joint2,double Joint3,double Joint4,double Joint5,double Joint6,double Joint7,double time,int path){
    SetRightMoveMode(0,path,time);
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=Joint1;
    msg.joint2=Joint2;
    msg.joint3=Joint3;
    msg.joint4=Joint4;
    msg.joint5=Joint5;
    msg.joint6=Joint6;
    msg.joint7=Joint7;
    RightJoint.publish(msg);
}

void Servo_Lib::MoveDRightEulerXYZ(double dx,double dy,double dz,double dalpha,double dbeta,double dgamma,double time,int path){
    SetRightMoveMode(1,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=RightPos[0]+dx;
    msg.Y_Axis=RightPos[1]+dy;
    msg.Z_Axis=RightPos[2]+dz;
    msg.alpha=RightEuler[0]+dalpha;
    msg.beta=RightEuler[1]+dbeta;
    msg.gamma=RightEuler[2]+dgamma;
    RightEulerXYZ.publish(msg);
}

void Servo_Lib::MoveDRightQuaternion(double dX,double dY,double dZ,double dx,double dy,double dz,double dw,double time,int path){
    SetRightMoveMode(1,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=RightPos[0]+dX;
    msg.Y_Axis=RightPos[1]+dY;
    msg.Z_Axis=RightPos[2]+dZ;
    msg.x=RightPos[3]+dx;
    msg.y=RightPos[4]+dy;
    msg.z=RightPos[5]+dz;
    msg.w=RightPos[6]+dw;
    RightQuaternion.publish(msg);
}

void Servo_Lib::MoveDRightJoint(double dJoint1,double dJoint2,double dJoint3,double dJoint4,double dJoint5,double dJoint6,double dJoint7,double time,int path){
    SetRightMoveMode(0,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=rightjoint[0]+dJoint1;
    msg.joint2=rightjoint[1]+dJoint2;
    msg.joint3=rightjoint[2]+dJoint3;
    msg.joint4=rightjoint[3]+dJoint4;
    msg.joint5=rightjoint[4]+dJoint5;
    msg.joint6=rightjoint[5]+dJoint6;
    msg.joint7=rightjoint[6]+dJoint7;
    RightJoint.publish(msg);
}

vector<double> Servo_Lib::GetRightEEPos(){
    ros::spinOnce();
    return RightPos;
}

/******************************************************************************************************/

/****************************************双臂相关函数****************************************************/
void Servo_Lib::MoveDualToHome(double time){
    SetRightMoveMode(0,0,time);
    SetLeftMoveMode(0,0,time);
    righthomeclient.call(homesrv);
    lefthomeclient.call(homesrv);
}

/******************************************************************************************************/

