/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 * Florent Lamiraux
 *
 * CNRS/AIST
 *
 * This file is part of abstract-robot-dynamics.
 * abstract-robot-dynamics is free software: you can redistribute it
 * and/or modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * abstract-robot-dynamics is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with abstract-robot-dynamics.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef JRL_ROBOT_DYNAMICS_OBJECT_CONSTRUCTOR
#define JRL_ROBOT_DYNAMICS_OBJECT_CONSTRUCTOR

#include "abstract-robot-dynamics/jrlHumanoidDynamicRobot.h"

/*! The creation of an object */
class CjrlRobotDynamicsObjectFactory
{
public:

  virtual ~CjrlRobotDynamicsObjectFactory()
    {};
  /**
     \brief Construct and return a pointer to a dynamic robot.
  */
  virtual CjrlDynamicRobot* createDynamicRobot()=0;

  /**
     \brief Construct and return a pointer to a humanoid dynamic robot.
  */
  virtual CjrlHumanoidDynamicRobot* createHumanoidDynamicRobot()=0;

  /**
     \brief Construct and return a pointer to a freeflyer joint.
     \param inInitialPosition position of the local frame of the joint when the robot is in initial configuration.

  */
  virtual CjrlJoint* createJointFreeflyer(const matrix4d& inInitialPosition)=0;

  /**
     \brief Construct and return a pointer to a rotation joint.
     \param inInitialPosition position of the local frame of the joint when the robot is in initial configuration.

  */
  virtual CjrlJoint* createJointRotation(const matrix4d& inInitialPosition)=0;

  /**
     \brief Construct and return a pointer to a translation joint.
     \param inInitialPosition position of the local frame of the joint when the robot is in initial configuration.

  */
  virtual CjrlJoint* createJointTranslation(const matrix4d& inInitialPosition)=0;

  /**
     \brief Construct and return a pointer to a body
  */
  virtual CjrlBody* createBody() = 0;

  /**
     \brief Construct and return a pointer to a hand.

     \param inWristJoint The joint the hand is attached to.

  */
  virtual CjrlHand* createHand(const CjrlJoint* inWristJoint) = 0;

  /**
     \brief Construct and return a pointer to a foot.

     \param inAnkle The joint the foot is attached to.
  */
  virtual CjrlFoot* createFoot(const CjrlJoint* inAnkle) = 0;


};

#endif

