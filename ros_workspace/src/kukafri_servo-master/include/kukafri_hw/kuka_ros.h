#ifndef _KUKA_ROS_H
#define _KUKA_ROS_H

#include <cstring>

#include "ros/ros.h"
#include "matrix.h"
#include "kuka_kinematics.h"

extern int GetCameraAxisParam(ros::NodeHandle &nh);
extern int GetHomeJointsParam(ros::NodeHandle &nh,std::string name);


#endif