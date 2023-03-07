/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Roboter GmbH, Augsburg, Germany.

SCOPE

The software KUKA Sunrise.Connectivity FRI Client SDK?is targeted to work in
conjunction with the KUKA Sunrise.Connectivity FastRobotInterface?toolkit.
In the following, the term Software?refers to all material directly
belonging to the provided SDK Software development kit? particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2017 
KUKA Roboter GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.13}
*/
#include <cstdlib>
#include <cstdio>
#include <cstring> // strstr
#include <math.h>

#include "MyLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include "kuka_ros.h"


#include "ros/ros.h"
#include "kukafri_hw/kukaState.h"
#include "kukafri_hw/kukaCmdJoint.h"
#include "kukafri_hw/kukaCmdPosQ.h"
#include "kukafri_hw/kukaCmdPosE.h"
#include "kukafri_hw/setMoveMode.h"
#include "kukafri_hw/moveToHome.h"
// #include "kukafri_hw/kukaCmdPosEOffset.h"


using namespace KUKA::FRI;


#define DEFAULT_PORTID 30200  //default is right's port
#define DEFAULT_IP "192.168.168.2" //default is right's IP
// #define _SIM //note the define if don't want to sim

// extern float FigTip_offset;
// void cmdPosQOffsetCallback(const kukafri_hw::kukaCmdPosEOffset::ConstPtr& _cmdPos,MyLBRClient*  _client,float* _figTipOffset){

//    if(_cmdPos->FigTip_offset<0||_cmdPos->FigTip_offset>0.3){
//       ROS_INFO("Input FigTipOffSet Out of Range !!!");
//    }else{
//       *_figTipOffset=_cmdPos->FigTip_offset;
//    }


//    MATRIX_D Quaternion=EulerZYX2Quaternion(_cmdPos->alpha*M_PI/180,_cmdPos->beta*M_PI/180,_cmdPos->gamma*M_PI/180);
//    _client->cmdPos[0]=_cmdPos->X_Axis;
//    _client->cmdPos[1]=_cmdPos->Y_Axis; 
//    _client->cmdPos[2]=_cmdPos->Z_Axis;
//    // _client->cmdPos[3]=_cmdPos->Roll*M_PI/180; _client->cmdPos[4]=_cmdPos->Pitch*M_PI/180; _client->cmdPos[5]=_cmdPos->Yaw*M_PI/180;
//    _client->cmdPos[3]=Quaternion(1,1); 
//    _client->cmdPos[4]=Quaternion(2,1); 
//    _client->cmdPos[5]=Quaternion(3,1); 
//    _client->cmdPos[6]=Quaternion(4,1);

//    for (int i = 0; i < 7; i++)
//    {
//       if(_client->cmdPos[i]!=_client->lastCmdPos[i])  
//       {  _client->newCmdFlag=true;
//          _client->posCmdFinishFlag=false;}
//    }
//    ROS_INFO("The eulerZYX alpha is set to %lf:",_cmdPos->alpha);
//    ROS_INFO("The eulerZYX beta  is set to %lf:",_cmdPos->beta);
//    ROS_INFO("The eulerZYX gamma is set to %lf:",_cmdPos->gamma);

//    ROS_INFO("The cmdPos is set to:");
//    for(int i=0;i<7;++i){
//       ROS_INFO("Pos%d[%lf]  ",i,_client->cmdPos[i]);
//    }
//    memcpy(_client->lastCmdPos,_client->cmdPos,7 * sizeof(double));

// }

void cmdJointCallback(const kukafri_hw::kukaCmdJoint::ConstPtr& _cmdJoint, MyLBRClient*  _client)  
{
   // ROS_INFO("I'm heard cmd is joint1:%lf, joint2:%lf, joint3:%lf, joint4:%lf, joint5:%lf, joint6:%lf, joint7:%lf\n",
   //          _cmdJoint->joint1,_cmdJoint->joint2,_cmdJoint->joint3,
   //          _cmdJoint->joint4,_cmdJoint->joint5,_cmdJoint->joint6,_cmdJoint->joint7);
   if (_cmdJoint->joint1>jointLimit[0]||_cmdJoint->joint2>jointLimit[1]||_cmdJoint->joint3>jointLimit[2]||_cmdJoint->joint4>jointLimit[3]
         ||_cmdJoint->joint5>jointLimit[4]||_cmdJoint->joint6>jointLimit[5]||_cmdJoint->joint7>jointLimit[6])
   {
      ROS_INFO("cmd joints out of range!! please try another cmd!");
      return;
   }

   _client->cmdJoints[0]=_cmdJoint->joint1*M_PI/180;
   _client->cmdJoints[1]=_cmdJoint->joint2*M_PI/180;
   _client->cmdJoints[2]=_cmdJoint->joint3*M_PI/180;
   _client->cmdJoints[3]=_cmdJoint->joint4*M_PI/180;
   _client->cmdJoints[4]=_cmdJoint->joint5*M_PI/180;
   _client->cmdJoints[5]=_cmdJoint->joint6*M_PI/180;
   _client->cmdJoints[6]=_cmdJoint->joint7*M_PI/180;

   // bool cmdchange=false;
   for (int i = 0; i < 7; i++)
   {
      if(_client->cmdJoints[i]!=_client->lastCmdJoints[i]) _client->newCmdFlag=true;
   }
   // if (cmdchange)
   // {
   //    _client->newCmdFlag=true;
   // }
   
   if(_client->newCmdFlag){
      ROS_INFO("The cmdJoints is set to:");
      for(int i=0;i<7;++i){
         ROS_INFO("joint%d[%lf]  ",i,_client->cmdJoints[i]);
      }
   }

   memcpy(_client->lastCmdJoints,_client->cmdJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
}
void cmdPosQCallback(const kukafri_hw::kukaCmdPosQ::ConstPtr& _cmdPos, MyLBRClient*  _client)  
{
   _client->cmdPos[0]=_cmdPos->X_Axis;
   _client->cmdPos[1]=_cmdPos->Y_Axis; 
   _client->cmdPos[2]=_cmdPos->Z_Axis;
   // _client->cmdPos[3]=_cmdPos->Roll*M_PI/180; _client->cmdPos[4]=_cmdPos->Pitch*M_PI/180; _client->cmdPos[5]=_cmdPos->Yaw*M_PI/180;
   _client->cmdPos[3]=_cmdPos->x; 
   _client->cmdPos[4]=_cmdPos->y; 
   _client->cmdPos[5]=_cmdPos->z; 
   _client->cmdPos[6]=_cmdPos->w;

   for (int i = 0; i < 7; i++)
   {
      if(_client->cmdPos[i]!=_client->lastCmdPos[i])  
      {  _client->newCmdFlag=true;
         _client->posCmdFinishFlag=false;}
   }
   ROS_INFO("The cmdPos is set to:");
   for(int i=0;i<7;++i){
      ROS_INFO("Pos%d[%lf]  ",i,_client->cmdPos[i]);
   }
   memcpy(_client->lastCmdPos,_client->cmdPos,7 * sizeof(double));

}
void cmdPosECallback(const kukafri_hw::kukaCmdPosE::ConstPtr& _cmdPos, MyLBRClient*  _client)  
{
   MATRIX_D Quaternion=EulerZYX2Quaternion(_cmdPos->alpha*M_PI/180,_cmdPos->beta*M_PI/180,_cmdPos->gamma*M_PI/180);
   _client->cmdPos[0]=_cmdPos->X_Axis;
   _client->cmdPos[1]=_cmdPos->Y_Axis; 
   _client->cmdPos[2]=_cmdPos->Z_Axis;
   // _client->cmdPos[3]=_cmdPos->Roll*M_PI/180; _client->cmdPos[4]=_cmdPos->Pitch*M_PI/180; _client->cmdPos[5]=_cmdPos->Yaw*M_PI/180;
   _client->cmdPos[3]=Quaternion(1,1); 
   _client->cmdPos[4]=Quaternion(2,1); 
   _client->cmdPos[5]=Quaternion(3,1); 
   _client->cmdPos[6]=Quaternion(4,1);

   for (int i = 0; i < 7; i++)
   {
      if(_client->cmdPos[i]!=_client->lastCmdPos[i])  
      {  _client->newCmdFlag=true;
         _client->posCmdFinishFlag=false;}
   }
   ROS_INFO("The eulerZYX alpha is set to %lf:",_cmdPos->alpha);
   ROS_INFO("The eulerZYX beta  is set to %lf:",_cmdPos->beta);
   ROS_INFO("The eulerZYX gamma is set to %lf:",_cmdPos->gamma);

   ROS_INFO("The cmdPos is set to:");
   for(int i=0;i<7;++i){
      ROS_INFO("Pos%d[%lf]  ",i,_client->cmdPos[i]);
   }
   memcpy(_client->lastCmdPos,_client->cmdPos,7 * sizeof(double));

}
bool setMoveMode(kukafri_hw::setMoveMode::Request& req, kukafri_hw::setMoveMode::Response& res, MyLBRClient*  _client)
{
   _client->moveMode = req.moveMode;
   _client->pathMode = req.pathMode;
   _client->moveDuration = (req.moveDuration>=0.2)?req.moveDuration:10;

   //ensure the robot will not move when moveMode change
   memcpy(_client->cmdJoints,_client->currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
   memcpy(_client->startJoints,_client->currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
   memcpy(_client->lastCmdJoints,_client->currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
   memcpy(_client->holdposJoints,_client->currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
   memcpy(_client->cmdPos,_client->currentPos,7 * sizeof(double));
   memcpy(_client->startPos,_client->currentPos,7 * sizeof(double));
   memcpy(_client->lastCmdPos,_client->currentPos,7 * sizeof(double));

   ROS_INFO("moveMode:[%d]; pathMode:[%d]; moveDuration:[%f]",_client->moveMode,_client->pathMode,_client->moveDuration);
   res.success=true;
   return true;
}
bool moveToHome(kukafri_hw::moveToHome::Request& req, kukafri_hw::moveToHome::Response& res, MyLBRClient*  _client)
{
   // int temp1=_client->moveMode,temp2=_client->pathMode;
   // double temp3=_client->moveDuration;

   // _client->moveMode = 0;
   // _client->pathMode = 0;
   // _client->moveDuration = 10;
   memcpy(_client->cmdJoints,homeJoints,7 * sizeof(double));
   _client->homeSrvFinishFlag = false;
   _client->newCmdFlag = true;
   memcpy(_client->lastCmdJoints,_client->cmdJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));

   // while(!_client->homeSrvFinishFlag){};

   // _client->moveMode = temp1;
   // _client->pathMode = temp2;
   // _client->moveDuration = temp3;
   //ensure the robot will not move when moveMode change
   // memcpy(_client->cmdJoints,_client->currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
   // memcpy(_client->startJoints,_client->currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
   // memcpy(_client->lastCmdJoints,_client->currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
   // memcpy(_client->holdposJoints,_client->currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
   // memcpy(_client->cmdPos,_client->currentPos,7 * sizeof(double));
   // memcpy(_client->startPos,_client->currentPos,7 * sizeof(double));
   // memcpy(_client->lastCmdPos,_client->currentPos,7 * sizeof(double));

   // ROS_INFO("moveMode:[%d]; pathMode:[%d]; moveDuration:[%f]",_client->moveMode,_client->pathMode,_client->moveDuration);
   res.success=true;
   return true;
}

int main (int argc, char** argv)
{
   //ros node init
   ros::init(argc, argv, "kukafri_hw");
   ros::NodeHandle nh;

   // read the arg when execuse
   const char * IP = (argc>=2) ? argv[1]:DEFAULT_IP;
   int PORT = (argc>=3) ? atoi(argv[2]):DEFAULT_PORTID;
   const char * name= (argc>=4) ? argv[3]:"right";
   const bool SIM = (argc>=5) ? atoi(argv[4]):true;

   std::string name_str=name;
   if(!GetHomeJointsParam(nh,name_str)){
      ROS_INFO("%s_home_joints is {0 ... 0}",name);
   }else{
      ROS_INFO("%s_home_joints get success",name);
   }

   //get camera calibration param
   if(!GetCameraAxisParam(nh)){
      ROS_INFO("base axis is %s_link_0",name);
   }else{
      Camera2Link0Trans(name);
      ROS_INFO("%s_arm_base axis change to camera sucess",name);
   }


   
   Gazebo_Sim GazeboSim(name_str);
   printf("gazebo_sim build ======\n");

   // create new client
   MyLBRClient client(SIM,&GazeboSim);
   printf("client build ======\n");

   // create new udp connection
   UdpConnection connection;

   // pass connection and client to a new FRI client application
   ClientApplication app(connection, client);


if(!SIM){// #ifndef _SIM
      printf("real robot start!\n");

      // Connect client application to KUKA Sunrise controller.
      // Parameter NULL means: repeat to the address, which sends the data
      // the second parameter is the robot-controller's IPaddress
      app.connect(PORT, IP);
   
      //init 'cmdJoints' and 'currentJoints' with current measured joints
      app.step();//only once step(), the sessionState change from 0 to 1, and the robot will not move
      memcpy(client.cmdJoints,client.robotState().getMeasuredJointPosition(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
      memcpy(client.lastCmdJoints,client.cmdJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
      memcpy(client.startJoints,client.cmdJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
      memcpy(client.currentJoints,client.robotState().getMeasuredJointPosition(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
}else{// #else
      printf("gazebo sim start!\n");

      // memcpy(client.cmdJoints,client.robotState().getMeasuredJointPosition(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
      GazeboSim.getCurJoints(client.cmdJoints);
      memcpy(client.lastCmdJoints,client.cmdJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
      memcpy(client.startJoints,client.cmdJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
      // memcpy(client.currentJoints,client.robotState().getMeasuredJointPosition(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
      GazeboSim.getCurJoints(client.currentJoints);
}// #endif
   
   memcpy(client.holdposJoints,client.currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
   memcpy(client.currentPos,KUKA_Kinematics2(client.currentJoints).matrix2array(),7 * sizeof(double));
   memcpy(client.cmdPos,client.currentPos,7 * sizeof(double));
   memcpy(client.lastCmdPos,client.cmdPos,7 * sizeof(double));
   memcpy(client.startPos,client.cmdPos,7 * sizeof(double));


   ROS_INFO("joint1:%lf, joint2:%lf, joint3:%lf, joint4:%lf, joint5:%lf, joint6:%lf, joint7:%lf\n",
            client.cmdJoints[0],client.cmdJoints[1],client.cmdJoints[2],client.cmdJoints[3],client.cmdJoints[4],client.cmdJoints[5],client.cmdJoints[6]);
 
    
   

   //ros msg init
   kukafri_hw::kukaState kukaState_;
   //init the pubmsg 'kukaState'
   for(int i=0;i<7;++i){
      kukaState_.Joints.push_back(0);
      kukaState_.Pos.push_back(0);
      if (i<3)
      {
         kukaState_.EulerZYX.push_back(0);
      }
   }
   // /*ros topic subscrible "kukaTipOffset"*/
   // ros::Subscriber sub_figTipOffset=nh.subscribe<kukafri_hw::kukaCmdPosEOffset>("kukaCmdPosEOffset",1,boost::bind(&cmdPosQOffsetCallback,_1,&client,&FigTip_offset)); 
   //ros topic advertise 'curState' init
   ros::Publisher pub_curState = nh.advertise<kukafri_hw::kukaState>("curState", 1);
   //ros topic subscribe 'cmdJoint' init
   ros::Subscriber sub_cmdJoint = nh.subscribe<kukafri_hw::kukaCmdJoint>("cmdJoint", 1 , boost::bind(&cmdJointCallback,_1,&client));
   //ros topic subscribe 'cmdPos_Quaternion'init
   ros::Subscriber sub_cmdPos_Q = nh.subscribe<kukafri_hw::kukaCmdPosQ>("cmdPos_Quaternion", 1 , boost::bind(&cmdPosQCallback,_1,&client));
   //ros topic subscribe 'cmdPos_EulerZYX' init
   ros::Subscriber sub_cmdPos_E = nh.subscribe<kukafri_hw::kukaCmdPosE>("cmdPos_EulerZYX", 1 , boost::bind(&cmdPosECallback,_1,&client));

   //ros server 'setMoveMode' init
   //  ros::ServiceServer srv_setMoveMode = nh.advertiseService<kukafri_hw::setMoveMode>("setMoveMode",boost::bind(&setMoveMode,_1,&client)); //这是错的:注意bind()的用法
    ros::ServiceServer srv_setMoveMode = nh.advertiseService<kukafri_hw::setMoveMode::Request,kukafri_hw::setMoveMode::Response>
                                                            ("setMoveMode",boost::bind(&setMoveMode,_1,_2,&client));
   //ros server 'MoveToHome' init
   ros::ServiceServer srv_MoveToHome = nh.advertiseService<kukafri_hw::moveToHome::Request,kukafri_hw::moveToHome::Response>
                                                            ("MoveToHome",boost::bind(&moveToHome,_1,_2,&client));
   //reset time for interpolation
   ResetTime();

   printf("start loop ======\n");

   // repeatedly call the step routine to receive and process FRI packets
   // repeatedly pub msg to topic 'curState'
   bool success = true;
   ros::Rate loop_rate(1000);
   while (ros::ok()&&success)
   {
      if(!SIM){// #ifndef _SIM
         success = app.step();
         memcpy(client.currentJoints,client.robotState().getMeasuredJointPosition(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
      }else{// #else
         client.command();
         GazeboSim.getCurJoints(client.currentJoints);
      }// #endif

      memcpy(client.currentPos,KUKA_Kinematics2(client.currentJoints).matrix2array(),7 * sizeof(double));
      memcpy(client.currentEuler, Quaternion2EulerZYX(client.currentPos[3],client.currentPos[4],
                                                      client.currentPos[5],client.currentPos[6]).matrix2array(), 3 * sizeof(double));

      for(int i=0;i<7;++i)
      {
         kukaState_.Joints[i]=client.currentJoints[i]*180/M_PI;
      //   if (i<3) kukaState_.Pos[i]=client.currentPos[i];
      //   if (i>=3&&i<6) kukaState_.Pos[i]=client.currentPos[i]*180/M_PI;
         kukaState_.Pos[i]=client.currentPos[i];
         if (i<3)
         {
            kukaState_.EulerZYX[i]=client.currentEuler[i]*180/M_PI;
         }
      }
      pub_curState.publish(kukaState_);
        


      ros::spinOnce();
      loop_rate.sleep();
   }

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Dispose                                                               */
   /*                                                                         */
   /***************************************************************************/
if(!SIM){// #ifndef _SIM
   // disconnect from controller
   app.disconnect();
}// #endif
   
   return 1;
}
