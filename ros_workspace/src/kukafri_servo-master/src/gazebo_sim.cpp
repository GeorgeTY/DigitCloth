
#include "gazebo_sim.h"


Gazebo_Sim::Gazebo_Sim(string name){
    side = (name=="right"?"R":"L");
    p = (name=="right"?9:1);
    cur_joints=nh.subscribe("/iiwa/joint_states",1,&Gazebo_Sim::sub_Joints,this);
    cmd_joints[0]=nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_"+side+"_J1_controller/command",1);
    cmd_joints[1]=nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_"+side+"_J2_controller/command",1);
    cmd_joints[2]=nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_"+side+"_J3_controller/command",1);
    cmd_joints[3]=nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_"+side+"_J4_controller/command",1);
    cmd_joints[4]=nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_"+side+"_J5_controller/command",1);
    cmd_joints[5]=nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_"+side+"_J6_controller/command",1);
    cmd_joints[6]=nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_"+side+"_J7_controller/command",1);

    ros::Duration(3).sleep();
}

Gazebo_Sim::~Gazebo_Sim(){
}

void Gazebo_Sim::sub_Joints(const sensor_msgs::JointState::ConstPtr& msg){
    joints=msg->position;
}

void Gazebo_Sim::getCurJoints(double* DesJoints){
    ros::spinOnce();
    memcpy(DesJoints,&joints[p],7*sizeof(double));
}

void Gazebo_Sim::moveGazebo(const double* RefTheta){
    std_msgs::Float64 msg;
    for (int i = 0; i < 7; i++)
    {
        msg.data=RefTheta[i];
        cmd_joints[i].publish(msg);
    }

}