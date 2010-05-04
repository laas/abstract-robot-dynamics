/*
 *   Copyright (c) 2006 CNRS-LAAS 
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Florent Lamiraux (LAAS-CNRS)
 *
 */

#ifndef JRL_ROBOT_DYNAMICS_OBJECT_CONSTRUCTOR
#define JRL_ROBOT_DYNAMICS_OBJECT_CONSTRUCTOR

#include "deprecated.h"
#include "jrlHumanoidDynamicRobot.h"

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

