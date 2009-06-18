/*
 *   Copyright (c) 2009 CNRS-LAAS 
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Florent Lamiraux (LAAS-CNRS)
 *
 */

#ifndef JRL_HUMDYROBOT_TYPE2_H
#define JRL_HUMDYROBOT_TYPE2_H

#include "jrlHumanoidDynamicRobot.h"

/**
   \brief Abstract class describing a humanoid robot with functional decomposition.
   
   This class derives for CjrlHumanoidDynamicRobot and provides more information about the structure of the robot. It provides list of joints corresponding to functional parts of a humanoid robot, namely
      \li the left arm, 
      \li the right arm, 
      \li the left leg, 
      \li the right leg, 
      \li the head.
*/

class CjrlHumDynRobotType2 : public virtual CjrlHumanoidDynamicRobot 
{
  virtual ~CjrlHumDynRobotType2() {};

  /** 
      \brief Returns the joints for one part of the body.
      
      \param inBodyPartIdentifier Name of the body part.
      \retval outListOfJoints List of joints correponding to this part of the robot.
      \return  false if the name of the body part is not supported.

      \note inBodyPartIdentifier should be one of the following:
      \li LEFTARM, 
      \li RIGHTARM, 
      \li LEFTLEG, 
      \li RIGHTLEG, 
      \li HEAD.
      
  */
  virtual bool GetBodyPartJoints(string inBodyPartIdentifier,
				 std::vector<Cjoint*> & outListOfJoints)=0;

};


#endif
