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

#ifndef JRL_HUMANOID_DYNAMIC_ROBOT
#define JRL_HUMANOID_DYNAMIC_ROBOT

#include "jrlDynamicRobot.h"
/** 
    \brief Abstract class that instantiate a humanoid robot with dynamics.

    This class derives for CjrlDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3> and instantiate properties specific to humanoid robots. 
    \li it provides pointers to the feet and hand joints,
    \li it provides pointers to the joint corresponding to the gaze,
    \li it computes the Zero Momentum Point.

    A humanoid robot is often in contact with the environment through
    one or both of its feet. Depending on the foot which is in contact
    with the ground, some computations using the dynamic model of the
    robot may differ. For this reason, the joints that are temporarily
    in contact with the environment are called fixed joints.
    
    A joint can be defined temporarily fixed by calling
    CjrlHumanoidDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3>::addFixedJoint. The joint is released by calling 
    CjrlHumanoidDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3>::removeFixedJoint.
*/

template <class Mnxp, class M4x4, class M3x3, class Vn, class V3> 
class CjrlHumanoidDynamicRobot : public CjrlDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3> {
public:
  /**
     \name Joints specific to humanoid robots
  */

  /**
     \brief Set the pointer to the left hand joint.
  */
  virtual void leftHand(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inLeftHand) = 0;

  /** 
      \brief Get a pointer to the left hand.
  */
  virtual CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* leftHand() = 0;

  /**
     \brief Set the pointer to the right hand joint.
  */
  virtual void rightHand(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inRightHand) = 0;

  /** 
      \brief Get a pointer to the right hand.
  */
  virtual CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* rightHand() = 0;

  /**
     \brief Set the pointer to the left foot joint.
  */
  virtual void leftFoot(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inLeftFoot) = 0;

  /** 
      \brief Get a pointer to the left foot.
  */
  virtual CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* leftFoot() = 0;

  /**
     \brief Set the pointer to the right foot joint.
  */
  virtual void rightFoot(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inRightFoot) = 0;

  /** 
      \brief Get a pointer to the right foot.
  */
  virtual CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* rightFoot() = 0;

  /** 
      \brief Set gaze joint
      
      \note  For most humanoid robots, the gaze joint is the head.
  */
  virtual void gazeJoint(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inGazeJoint) = 0;

  /**
     \brief Get gaze joint
  */
  virtual CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* gazeJoint() = 0;

  /**
     \brief Set the gaze in the local frame of the gaze joint.

     \note The gaze is defined as a straight line linked to the gaze joint.
  */
  virtual void gaze(const V3& inStraightLine) = 0;

  /** 
      \brief Get the gaze orientation in the local frame of the gaze joint.
      \return outOrigin a point on the gaze straight line,
      \return outDirection the direction of the gaze joint.
  */
  virtual void gaze(V3& outOrigin, V3& outDirection) const = 0;

  /** 
      \brief Add a joint to the vector of fixed joints.
       This Declares a joint as fixed in the world.
  */
  virtual void addFixedJoint(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inFixedJoint) = 0;

 /** 
      \brief Count joints that are fixed in the world.
  */
  virtual unsigned int countFixedJoints() const = 0;

 /** 
      \brief Remove a joint from the vector of fixed joints.
       The input joint will no longer be considered fixed in the world.
  */
  virtual void removeFixedJoint(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inFixedJoint) = 0;

 /** 
      \brief Return the fixed joint at rank inRank 
  */
  virtual const CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>& fixedJoint(unsigned int inJointRank) const = 0;

 /**
    \brief Get the jacobian of a joint wrt to internal configuration variables assuming a joint is fixed.
    
     Fixed joint is first fixed joint in vector.
     \return true if there is at least one fixed joint, false otherwise.  
  */
  virtual bool jacobianJointWrtFixedJoint(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inJoint, Mnxp& outJacobian) = 0;
  
  /** 
  \brief Return the distance between the sole of a foot and its joint center
   */
  virtual double footHeight() const = 0;



  /**
     \@}
  */

    /**
     \name Zero momentum point
  */

  /**
     \brief Compute the coordinates of the Zero Momentum Point.

  /**
     \brief Get the coordinates of the Zero Momemtum Point.
  */
  virtual const V3& zeroMomentumPoint() const = 0;


  /**
     @}
  */

};

#endif
