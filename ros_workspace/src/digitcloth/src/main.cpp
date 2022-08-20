#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include <string>



using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "digitcloth");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}