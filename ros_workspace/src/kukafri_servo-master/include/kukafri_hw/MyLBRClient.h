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
#ifndef _KUKA_FRI_MY_LBR_CLIENT_H
#define _KUKA_FRI_MY_LBR_CLIENT_H

#include "friLBRClient.h"
#include "calc_pathInterpolation.h"
#include "system_time.h"
#include "kuka_kinematics.h"
#include "gazebo_sim.h"



/**
 * \brief Template client implementation.
 */
class MyLBRClient : public KUKA::FRI::LBRClient
{
   
public:
      
   /**
    * \brief Constructor.
    */
   // MyLBRClient();
   MyLBRClient(const bool,Gazebo_Sim*);

   /** 
    * \brief Destructor.
    */
   ~MyLBRClient();
   
   /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);
   
   /**
    * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
    * 
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
   virtual void monitor();
   
   /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    * 
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
   virtual void waitForCommand();
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    * 
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
   virtual void command();

   double currentJoints[7];
   double cmdJoints[7];
   double lastCmdJoints[7];
   double startJoints[7];

   double currentPos[7];
   double cmdPos[7];
   double lastCmdPos[7];
   double startPos[7];
   double holdposJoints[7];

   double currentEuler[3];
   // double cmdEuler[3];
   // double IpoJoints[7]; //IpoJoints can not change ??
   // double callbackJointTest[7];

   int moveMode;
   int pathMode;
   double moveDuration; //interpolation frequence
   bool newCmdFlag;
   bool posCmdFinishFlag;
   bool homeSrvFinishFlag;

   bool SIM;
   Gazebo_Sim* gazeboSim=NULL;
};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
