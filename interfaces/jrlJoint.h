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
     \brief Get the velocity \f$({\bf v}, {\bf \omega})\f$ of the joint.

     The velocity is determined by the configuration of the robot and its time derivative: \f$({\bf q},{\bf \dot{q}})\f$.
     
     \return the linear velocity \f${\bf v}\f$ of the origin of the joint frame
     and the angular velocity \f${\bf \omega}\f$ of the joint frame.
  */
  virtual CjrlRigidVelocity jointVelocity()=0;

  /**
     \brief Get the acceleration of the joint.

     The acceleratoin is determined by the configuration of the robot and its first and second time derivative: \f$({\bf q},{\bf \dot{q}}, {\bf \ddot{q}})\f$.
  */
  virtual CjrlRigidAcceleration jointAcceleration()=0;

  /**
     \brief Get the number of degrees of freedom of the joint.
  */
  virtual unsigned int numberDof() const=0;
 	

  /**
     @}
  */

  /**
     \name Jacobian functions wrt configuration.
     @{
  */

  /**
     \brief Get the Jacobian matrix of the joint position and orientation wrt the robot configuration.
     Kinematical constraints from interaction with the environment are not taken into account for this computation.

     The corresponding computation can be done by the robot for each of its joints or by the joint.
     
     \return a matrix \f$J \in {\bf R}^{6\times n_{dof}}\f$ defined by 
     \f[
     J = \left(\begin{array}{llll}
     {\bf v_1} & {\bf v_2} & \cdots & {\bf v_{n_{dof-6}}} \\
     {\bf \omega_1} & {\bf \omega_2} & \cdots & {\bf \omega_{n_{dof-6}}}
     \end{array}\right)
     \f]
     where \f${\bf v_i}\f$ and \f${\bf \omega_i}\f$ are respectively the linear and angular velocities of the joint 
     implied by the variation of degree of freedom \f$q_i\f$. The velocity of the joint returned by 
     CjrlJoint::jointVelocity can thus be obtained through the following formula:
     \f[
     \left(\begin{array}{l} {\bf v} \\ {\bf \omega}\end{array}\right) = J {\bf \dot{q}}
     \f]
  */
  virtual ublas::matrix jacobianJointWrtConfig() const = 0;

 /**
     \brief Compute the joint's jacobian wrt the robot configuration.
  */
  virtual computeJacobianJointWrtConfig() = 0;

 /**
     \brief Get the jacobian of the point specified in local frame by inPointJointFrame.
	
  */
  virtual ublas::matrix<double> jacobianPointWrtConfig(ublas::vector inPointJointFrame) const = 0;

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
