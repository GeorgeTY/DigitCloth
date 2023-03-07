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
#include <cstdio>
#include "MyLBRClient.h"

using namespace KUKA::FRI;
//******************************************************************************
// MyLBRClient::MyLBRClient()
// :moveMode(0),pathMode(0),moveDuration(10.0),newCmdFlag(false),posCmdFinishFlag(true)
// {
// }

MyLBRClient::MyLBRClient(const bool sim, Gazebo_Sim* gazebosim)
:moveMode(0),pathMode(0),moveDuration(10.0),newCmdFlag(false),posCmdFinishFlag(true)
{
   SIM=sim;
   if(sim) gazeboSim=gazebosim;
}

//******************************************************************************
MyLBRClient::~MyLBRClient()
{
}
      
//******************************************************************************
void MyLBRClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // react on state change events
   switch (newState)
   {
      case MONITORING_WAIT:
      {
         break;
      }       
      case MONITORING_READY:
      {
         break;
      }
      case COMMANDING_WAIT:
      {
         break;
      }   
      case COMMANDING_ACTIVE:
      {
         break;
      }   
      default:
      {
         break;
      }
   }
}

//******************************************************************************
void MyLBRClient::monitor()
{
   LBRClient::monitor();
   
   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/
   
}

//******************************************************************************
void MyLBRClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, 
   // by calling the base method.
   LBRClient::waitForCommand();
   
   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/   
   
}

//******************************************************************************
void MyLBRClient::command()
{
   double RefTheta[7];
   double RefPos[7];
   
   if (newCmdFlag)//when new move is commanded
   {  
      if (!moveMode){//if jointdegree position move mode,
         memcpy(startJoints,currentJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));
      }
      else {//if endeffect position move mode, 
         memcpy(startPos , currentPos, 7 * sizeof(double));
      }
      SetCurtimeasStartTime();
      
      newCmdFlag=false;
   }
   
   if (!moveMode)//joint degree move
   {
         switch (pathMode)
         {
         case INTERPOLATION_ONE:
            for (int i = 0; i < 7; i++)
            {
               RefTheta[i]=Calc1JiTraje(startJoints[i],cmdJoints[i],moveDuration,GetOffsetTime(),homeSrvFinishFlag);
               // printf("offsettime:%lf\n",GetOffsetTime());
            }

            break;
         case INTERPOLATION_STEP:
            for (int i = 0; i < 7; i++)
            {
               RefTheta[i]=CalcStepTraje(startJoints[i],cmdJoints[i]);
            }
            break;
         case INTERPOLATION_SIN:
            for (int i = 0; i < 7; i++)
            {
               RefTheta[i]=CalcSinTraje(startJoints[i],cmdJoints[i],moveDuration,GetOffsetTime(),homeSrvFinishFlag);
            }
            break;

         default:
            for (int i = 0; i < 7; i++)
            {
               RefTheta[i]=Calc1JiTraje(startJoints[i],cmdJoints[i],moveDuration,GetOffsetTime(),homeSrvFinishFlag);
            }
            break;
         }
         // double IpoJoints[7];//IpoJoints will not change ???
         // memcpy(IpoJoints,MyLBRClient::robotState().getIpoJointPosition(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
         // printf("\nIpoJoints:\n"); for (int i = 0; i < 7; i++){ printf("[%lf] ",MyLBRClient::robotState().getIpoJointPosition()[i]*180/M_PI);}//for test  
         // printf("\nIpoJoints:\n"); for (int i = 0; i < 7; i++){ printf("[%lf] ",IpoJoints[i]*180/M_PI);}//for test  
   }
   else //endeffect position move
   {
      // printf("posmode switch\n");
         switch (pathMode)
         {
         // case INTERPOLATION_ONE:
         //    for (int i = 0; i < 6; i++)
         //    {
         //       RefPos[i]=Calc1JiTraje(startPos[i],cmdPos[i],moveDuration,GetOffsetTime(),posCmdFinishFlag);
         //       // printf("offsettime:%lf\n",GetOffsetTime());
         //    }

         //    break;
         // case INTERPOLATION_STEP: //step is not safe, so step there equals to noce_interpolation
         //    for (int i = 0; i < 6; i++)
         //    {
         //       // RefTheta[i]=CalcStepTraje(startPos[i],cmdPos[i]);
         //       RefPos[i]=Calc1JiTraje(startPos[i],cmdPos[i],moveDuration,GetOffsetTime(),posCmdFinishFlag);
         //    }
         //    break;
         // case INTERPOLATION_SIN:
         //    for (int i = 0; i < 6; i++)
         //    {
         //       RefPos[i]=CalcSinTraje(startPos[i],cmdPos[i],moveDuration,GetOffsetTime(),posCmdFinishFlag);
         //    }
         //    break;

         default:
            // for (int i = 0; i < 6; i++)
            // {
            //    RefPos[i]=Calc1JiTraje(startPos[i],cmdPos[i],moveDuration,GetOffsetTime(),posCmdFinishFlag);
            // }
      // printf("posmode interpolation\n");
      // for (int i = 0; i < 7; i++)
      // {
      //    printf("RefPos[%d]:%f",i,RefPos[i]);
      // }
            memcpy(RefPos,CalcSlerp(startPos,cmdPos,moveDuration,GetOffsetTime(),posCmdFinishFlag).matrix2array(), 7 * sizeof(double));
      // for (int i = 0; i < 7; i++)
      // {
      //    printf("RefPos[%d]:%f",i,RefPos[i]);
      // }
            break;
         }
      
         printf("\ncurrentJoints:\n"); for (int i = 0; i < 7; i++){ printf("[%lf] ",currentJoints[i]*180/M_PI);}//for test
         printf("\ncurrentPos:\n"); for (int i = 0; i < 7; i++){ printf("[%lf] ",currentPos[i]);}//for test
         printf("\nstartPos:\n");   for (int i = 0; i < 7; i++){ printf("[%lf] ",startPos[i]);}//for test
         printf("\ncmdPos:\n");     for (int i = 0; i < 7; i++){ printf("[%lf] ",cmdPos[i]);}//for test
         printf("\nRefPos:\n");     for (int i = 0; i < 7; i++){ printf("[%lf] ",RefPos[i]);}//for test
         printf("\n");

         memcpy(RefTheta,KUKA_InKinematics(RefPos,currentPos,currentJoints,holdposJoints,posCmdFinishFlag).matrix2array(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
         memcpy(holdposJoints,RefTheta,LBRState::NUMBER_OF_JOINTS * sizeof(double));

         // memcpy(currentJoints,RefTheta,LBRState::NUMBER_OF_JOINTS * sizeof(double));//for test
         // memcpy(RefTheta,cmdJoints,LBRState::NUMBER_OF_JOINTS * sizeof(double));//for test
         printf("\nRefTheta:\n"); for (int i = 0; i < 7; i++){ printf("[%lf] ",RefTheta[i]*180/M_PI);}//for test
         
   }

   
   // for (int i = 0; i < 7; i++)
   // {
   //    printf("RefTheta:%f  ",RefTheta[i]*180/M_PI);
   // }
   // printf("\n");
   
   
   

   // printf("\ncurrentJoints:\n"); for (int i = 0; i < 7; i++){ printf("[%lf] ",currentJoints[i]);}//for test
   // In command(), the joint angle values have to be set. 
   // robotCommand().setJointPosition( currentJoints );//for test //**DANGEROUS!! **
   // robotCommand().setJointPosition( cmdJoints );//for test //**it is safe WITHOUT pub to /cmdJoint !!!!**
   if(!SIM){
      robotCommand().setJointPosition( RefTheta );
   }else{
      gazeboSim->moveGazebo(RefTheta);
   }
}
