/*
 *   Copyright (c) 2006 CNRS-LAAS 
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Developed by Florent Lamiraux (LAAS-CNRS)
 *
 */

#ifndef JRL_HUMANOID_DYNAMIC_ROBOT
#define JRL_HUMANOID_DYNAMIC_ROBOT

/** 
    \brief Abstract class that instantiate a humanoid robot with dynamics.

    This class derives for CjrlDynamicRobot and instantiate properties specific to humanoid robots. 
    \li it provides pointers to the feet and hand joints,
    \li it provides pointers to the joint corresponding to the gaze,
    \li it computes the Zero Momentum Point.
*/

class CjrlHumanoidDynamicRobot : public CjrlDynamicRobot {
public:
  /**
     \name Joints specific to humanoid robots
  */

  /**
     \brief Set the pointer to the left hand joint.
  */
  virtual void leftHand(CjrlJoint* inLeftHand) = 0;

  /** 
      \brief Get a pointer to the left hand.
  */
  virtual CjrlJoint* leftHand() = 0;

  /**
     \brief Set the pointer to the right hand joint.
  */
  virtual void rightHand(CjrlJoint* inRightHand) = 0;

  /** 
      \brief Get a pointer to the right hand.
  */
  virtual CjrlJoint* rightHand() = 0;

  /**
     \brief Set the pointer to the left foot joint.
  */
  virtual void leftFoot(CjrlJoint* inLeftFoot) = 0;

  /** 
      \brief Get a pointer to the left foot.
  */
  virtual CjrlJoint* leftFoot() = 0;

  /**
     \brief Set the pointer to the right foot joint.
  */
  virtual void rightFoot(CjrlJoint* inRightFoot) = 0;

  /** 
      \brief Get a pointer to the right foot.
  */
  virtual CjrlJoint* rightFoot() = 0;

  /** 
      \brief Set gaze joint
      
      \note  For most humanoid robots, the gaze joint is the head.
  */
  virtual void gazeJoint(CjrlJoint* inGazeJoint) = 0;

  /**
     \brief Get gaze joint
  */
  virtual Cjrljoint* gazeJoint() = 0;

  /**
     \brief Set the gaze in the local frame of the gaze joint.

     \note The gaze is defined as a straight line linked to the gaze joint.
  */
  virtual void gaze(const Cjrl3dStraightLine& inStraightLine) = 0;

  /** 
      \brief Get the gaze orientation in the local frame of the gaze joint.
  */
  virtual const Cjrl3dStraightLine& gaze() const = 0;

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
  virtual const vector3d& zeroMomentumPoint() const = 0;


  /**
     @}
  */

};

#endif
