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

template <CdynamicRobot, ChumanoidDynamicRobot, CjointFreeflyer, CjointRotation, CjointTranslation, Cbody, Mnxp, M4x4, M3x3, Vn, V3> 
  class CjrlRobotDynamicsObjectConstructor
{
public:
  /**
     \brief Construct and return a pointer to a dynamic robot.
  */
  static CjrlDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3>* createDynamicRobot() {return new CdynamicRobot;};

  /**
     \brief Construct and return a pointer to a humanoid dynamic robot.
  */
  static CjrlHumanoidDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3>* createhumanoidDynamicRobot() {return new ChumanoidDynamicRobot();};

  /**
     \brief Construct and return a pointer to a freeflyer joint.
     \param inInitialPosition position of the local frame of the joint when the robot is in initial configuration.
     
  */
  static CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* createJointFreeflyer(const M4x4& inInitialPosition) {return new CjointFreeflyer(inInitialPosition);};

  /**
     \brief Construct and return a pointer to a rotation joint.
     \param inInitialPosition position of the local frame of the joint when the robot is in initial configuration.
     
  */
  static CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* createJointRotation(const M4x4& inInitialPosition) {return new CjointRotation(inInitialPosition);};

  /**
     \brief Construct and return a pointer to a translation joint.
     \param inInitialPosition position of the local frame of the joint when the robot is in initial configuration.
     
  */
  static CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* createJointTranslation(const M4x4& inInitialPosition) {return new CjointTranslation(inInitialPosition);};

  /**
     \brief Construct and return a pointer to a body
  */
  static CjrlBody* createBody() {return new Cbody();};
};

#endif

