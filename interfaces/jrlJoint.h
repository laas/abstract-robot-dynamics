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

#ifndef JRL_JOINT_H
#define JRL_JOINT_H

/**
   \brief This class represents a robot joint.

   A joint may have several degrees of freedom. The position, velocity
   and acceleration of a joint are defined by the configuration vector
   \f${\bf q}\f$ and its first and second derivatives.
*/
class CjrlJoint {
public:

  /**
     \name Joint hierarchy
     @{
  */

  /**
     \brief Get a pointer to the parent joint (if any).
  */
  virtual CjrlJoint& parentJoint() const=0;

  /**
     \brief Add a child joint.
  */
  virtual bool addChildJoint (const CjrlJoint& inJoint)=0;

  /**
     \brief Get the number of children.
  */
  virtual unsigned int countChildJoints() const=0;

  /**
     \brief  	Returns the child joint at the given rank.
  */
  virtual const CjrlJoint& childJoint(unsigned int inJointRank) const=0;

  /**
     \brief Get a vector containing references of the joints between the rootJoint and this joint.
  */
  virtual std::vector<CjrlJoint&> jointsFromRootToThis() const = 0;

  /**
     @}
  */

  /**
     \name Joint kinematics
     @{
  */
  
  /**
     \brief Get the current transformation of the joint.
     
     The current transformation of the joint is the transformation
     moving the joint from the position in initial configuration to
     the current position. 
     
     The current transformation is determined by the configuration \f${\bf q}\f$ of the robot.
  */
  virtual const matrix4d &currentTransformation() const=0;

  /**
     \brief Get the velocity of the joint.

     The velocity is determined by the configuration of the robot and its time derivative: \f$({\bf q},{\bf \dot{q}})\f$.
  */
  virtual CjrlRigidVelocity getGetJointVelocity()=0;

  /**
     \brief Get the acceleration of the joint.

     The acceleratoin is determined by the configuration of the robot and its first and second time derivative: \f$({\bf q},{\bf \dot{q}}, {\bf \ddot{q}})\f$.
  */
  virtual CjrlRigidAcceleration getGetJointAcceleration()=0;

  /**
     \brief Get the number of degrees of freedom of the joint.
  */
  virtual unsigned int getNumberDof() const=0;
 	

  /**
     @}
  */

  /**
     \name Body linked to the joint
     @{
  */

  /**
     \brief Get a pointer to the linked body (if any).
  */
  virtual CjrlBody* linkedBody() const = 0;
 	
  /**
     \brief Link a body to the joint.
  */
  virtual ktStatus setLinkedBody (CjrlBody& inBody) = 0;
  
  /**
     @}
  */
  
};


#endif
