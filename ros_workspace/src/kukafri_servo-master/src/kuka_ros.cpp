
#include "kuka_ros.h"


int GetCameraAxisParam(ros::NodeHandle &nh)
{
    int success=1;
    if(!nh.getParam("/camera_axis/pos/x",cameraXYZ(1,1))) success=0;
    if(!nh.getParam("/camera_axis/pos/y",cameraXYZ(2,1))) success=0;
    if(!nh.getParam("/camera_axis/pos/z",cameraXYZ(3,1))) success=0;
    if(!nh.getParam("/camera_axis/rot/Rx",cameraQuaternion(1,1))) success=0;
    if(!nh.getParam("/camera_axis/rot/Ry",cameraQuaternion(2,1))) success=0;
    if(!nh.getParam("/camera_axis/rot/Rz",cameraQuaternion(3,1))) success=0;
    if(!nh.getParam("/camera_axis/rot/Rw",cameraQuaternion(4,1))) success=0;
    std::string baselink0;
    if(!nh.getParam("/camera_axis/base_link0",baselink0)) success=0;

//one or more param don't get 
    if(!success){
        cameraXYZ=MatD31(0,0,0);
        cameraQuaternion=MatD41(0,0,0,1);
        return success;
    }

//all param get successfully
    for (int i = 1; i <= 3; i++)
    {
        cameraQuaternion(i,1)= (success?(-cameraQuaternion(i,1)):0);
    }
    cameraQuaternion(4,1)=(success?(cameraQuaternion(4,1)):1);
    camera_base_link0=baselink0.c_str();
    // printf("%s",camera_base_link0);
    return success;
}

int GetHomeJointsParam(ros::NodeHandle &nh,std::string name)
{
    int success=1;
    if(!nh.getParam("/home_joints/"+name+"/joint1",homeJoints[0])) success=0;
    if(!nh.getParam("/home_joints/"+name+"/joint2",homeJoints[1])) success=0;
    if(!nh.getParam("/home_joints/"+name+"/joint3",homeJoints[2])) success=0;
    if(!nh.getParam("/home_joints/"+name+"/joint4",homeJoints[3])) success=0;
    if(!nh.getParam("/home_joints/"+name+"/joint5",homeJoints[4])) success=0;
    if(!nh.getParam("/home_joints/"+name+"/joint6",homeJoints[5])) success=0;
    if(!nh.getParam("/home_joints/"+name+"/joint7",homeJoints[6])) success=0;

    if (!success)
    {
        for (int i = 0; i < 7; i++)
        {
            homeJoints[i]=0;
        }
        return success;
    }

    for (int i = 0; i < 7; i++)
    {
        // printf("%lf  ",homeJoints[i]);
        homeJoints[i]=homeJoints[i]*M_PI/180;
    }
    
    return success;
    

}

