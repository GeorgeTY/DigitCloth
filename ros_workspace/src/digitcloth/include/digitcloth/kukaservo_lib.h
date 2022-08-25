#ifndef SERVO
#define SERVO

#include <ros/ros.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <algorithm> 

#include "kukafri_hw/setMoveMode.h"
#include "kukafri_hw/moveToHome.h"
#include "kukafri_hw/kukaCmdPosE.h"
#include "kukafri_hw/kukaCmdPosQ.h"
#include "kukafri_hw/kukaCmdJoint.h"
#include "kukafri_hw/kukaState.h"

using namespace std;

class Servo_Lib{
    private:
        vector<double> LeftPos;
        vector<double> LeftEuler;
        vector<double> leftjoint;
        vector<double> RightPos;
        vector<double> RightEuler;
        vector<double> rightjoint;

        ros::NodeHandle nh;
        ros::ServiceClient leftclient;
        ros::ServiceClient rightclient;
        ros::ServiceClient lefthomeclient;
        ros::ServiceClient righthomeclient;
                
        ros::Publisher LeftEulerXYZ;
        ros::Publisher LeftQuaternion;
        ros::Publisher LeftJoint;
        ros::Publisher RightEulerXYZ;
        ros::Publisher RightQuaternion;
        ros::Publisher RightJoint;

        ros::Subscriber LeftKukaPos;
        ros::Subscriber RightKukaPos;

        kukafri_hw::moveToHome homesrv;

        void leftkukaCb(const kukafri_hw::kukaState::ConstPtr& msg);
        void rightkukaCb(const kukafri_hw::kukaState::ConstPtr& msg);

    public:
        Servo_Lib();
        ~Servo_Lib();

        /*返回Lefthome位置*/
        void MoveLeftToHome(double time=10);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       alpha为沿X轴旋转到的目标角度    beta为沿Y轴旋转到的目标角度   gamma为沿Z轴旋转到的目标角度*/
        void MoveLeftEulerXYZ(double X_Axis=0.6,double Y_Axis=0.225,double Z_Axis=0.669,double alpha=0,double beta=0,double gamma=-90,double time=10,int path=0);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       x,y,z,w为目标点四元数*/
        void MoveLeftQuaternion(double X_Axis=0.6,double Y_Axis=0.225,double Z_Axis=0.669,double x=0,double y=0,double z=-0.707,double w=0.707,double time=10,int path=0);
        /*指定机械臂七个关节角进行移动*/
        void MoveLeftJoint(double Joint1=-23.09,double Joint2=-29.527,double Joint3=4.9144,double Joint4=-72.56458,double Joint5=25.49335,double Joint6=46.8971,double Joint7=9.2455,double time=10,int path=0);
        /*机械臂相对于当前位置移动一个坐标和位姿,位姿用EulerXYZ表示*/
        void MoveDLeftEulerXYZ(double dx=0,double dy=0,double dz=0,double dalpha=0,double dbeta=0,double dgamma=0,double time=10,int path=0);
        /*机械臂相对于当前位置移动一个坐标和位姿，位姿用四元数表示*/
        void MoveDLeftQuaternion(double dX=0,double dY=0,double dZ=0,double dx=0,double dy=0,double dz=0,double dw=0,double time=10,int path=0);
        /*机械臂各关节角相对于当前关节角移动一个关节角,单位为°*/
        void MoveDLeftJoint(double dJoint1=0,double dJoint2=0,double dJoint3=0,double dJoint4=0,double dJoint5=0,double dJoint6=0,double dJoint7=0,double time=10,int path=0);
        /*设置运动模式*/
        void SetLeftMoveMode(int moveMode=0,int pathMode=0,double moveDuration=10);
        

        /*返回Righthome位置*/
        void MoveRightToHome(double time=10);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       alpha为沿X轴旋转到的目标角度    beta为沿Y轴旋转到的目标角度   gamma为沿Z轴旋转到的目标角度*/
        void MoveRightEulerXYZ(double X_Axis=-0.6,double Y_Axis=0.225,double Z_Axis=0.7,double alpha=0,double beta=0,double gamma=-90,double time=10,int path=0);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       x,y,z,w为目标点四元数*/
        void MoveRightQuaternion(double X_Axis=-0.6,double Y_Axis=0.225,double Z_Axis=0.7,double x=0,double y=0,double z=-0.707,double w=0.707,double time=10,int path=0);
        /*指定机械臂七个关节角进行移动*/
        void MoveRightJoint(double Joint1=28.6085,double Joint2=-29.537,double Joint3=-6.1648,double Joint4=-72.56458,double Joint5=-29.92,double Joint6=48.27,double Joint7=-8.845,double time=10,int path=0);
        /*机械臂相对于当前位置移动一个坐标和位姿,位姿用EulerXYZ表示*/
        void MoveDRightEulerXYZ(double dx=0,double dy=0,double dz=0,double dalpha=0,double dbeta=0,double dgamma=0,double time=10,int path=0);
        /*机械臂相对于当前位置移动一个坐标和位姿，位姿用四元数表示*/
        void MoveDRightQuaternion(double dX=0,double dY=0,double dZ=0,double dx=0,double dy=0,double dz=0,double dw=0,double time=10,int path=0);
        /*机械臂各关节角相对于当前关节角移动一个关节角,单位为°*/
        void MoveDRightJoint(double dJoint1=0,double dJoint2=0,double dJoint3=0,double dJoint4=0,double dJoint5=0,double dJoint6=0,double dJoint7=0,double time=10,int path=0);
        /*设置运动模式*/
        void SetRightMoveMode(int moveMode=0,int pathMode=0,double moveDuration=10);
        /*确保运动模式为末端位姿运动时，使用这个函数做伺服--重载的函数*/
        void MoveRightEulerXYZ(int flag,double X_Axis=-0.6,double Y_Axis=0.225,double Z_Axis=0.7,double alpha=0,double beta=0,double gamma=-90);
        /*获取右臂末端位姿信息*/
        vector<double> GetRightEEPos();

        /*返回Dualhome位置*/
        void MoveDualToHome(double time=10);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       alpha为沿X轴旋转到的目标角度    beta为沿Y轴旋转到的目标角度   gamma为沿Z轴旋转到的目标角度*/
        void MoveDualEulerXYZ(double X_Axis=-0.6,double Y_Axis=0.225,double Z_Axis=0.7,double alpha=0,double beta=0,double gamma=-90,double time=10,int path=0);
};

#endif