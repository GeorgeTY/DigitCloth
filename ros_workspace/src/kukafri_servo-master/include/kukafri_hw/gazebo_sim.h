//std rosmsg: use for gazebo sim 
#ifndef _GAZEBO_SIM_H
#define _GAZWBO_SIM_H

#include <ros/ros.h>
#include <vector>
#include <string>

#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

using namespace std;

class Gazebo_Sim{
    private:
        vector<double> joints={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        string side;//right:true; left:false
        int p;

        ros::NodeHandle nh;
        ros::Subscriber cur_joints;
        ros::Publisher cmd_joints[7];
        // ros::Publisher cmd_joints2;
        // ros::Publisher cmd_joints3;
        // ros::Publisher cmd_joints4;
        // ros::Publisher cmd_joints5;
        // ros::Publisher cmd_joints6;
        // ros::Publisher cmd_joints7;


        void sub_Joints(const sensor_msgs::JointState::ConstPtr& msg);

    public:
        Gazebo_Sim(string);
        ~Gazebo_Sim();

        void moveGazebo(const double*);
        void getCurJoints(double*);


};

#endif