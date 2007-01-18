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

template <CdynamicRobot, ChumanoidDynamicRobot, CjointFreeflyer, CjointRotation, CjointTranslation, Cbody> 
  class CjrlRobotDynamicsObjectConstructor
{
public:
  /**
     \brief Construct and return a pointer to a dynamic robot.
  */
  static CjrlDynamicRobot* createDynamicRobot() {return new CdynamicRobot;};

  /**
     \brief Construct and return a pointer to a humanoid dynamic robot.
  */
  static CjrlHumanoidDynamicRobot* createhumanoidDynamicRobot() {return new ChumanoidDynamicRobot();};

  /**
     \brief Construct and return a pointer to a freeflyer joint.
     \param inInitialPosition position of the local frame of the joint when the robot is in initial configuration.
     
  */
  static CjrlJoint* createJointFreeflyer(const matrix4d& inInitialPosition) {return new CjointFreeflyer(inInitialPosition);};

  /**
     \brief Construct and return a pointer to a rotation joint.
     \param inInitialPosition position of the local frame of the joint when the robot is in initial configuration.
     
  */
  static CjrlJoint* createJointRotation(const matrix4d& inInitialPosition) {return new CjointRotation(inInitialPosition);};

  /**
     \brief Construct and return a pointer to a translation joint.
     \param inInitialPosition position of the local frame of the joint when the robot is in initial configuration.
     
  */
  static CjrlJoint* createJointTranslation(const matrix4d& inInitialPosition) {return new CjointTranslation(inInitialPosition);};

  /**
     \brief Construct and return a pointer to a body
  */
  static CjrlBody* createBody() {return new Cbody();};
};

#endif

