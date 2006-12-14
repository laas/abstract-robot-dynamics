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

#ifndef JRL_DYNAMIC_ROBOT
#define JRL_DYNAMIC_ROBOT

/**
   \brief Abstract class that instantiate a robot with dynamic properties.

   A robot is a kinematic chain of joints. To each joint is attached a body
   Each body has a mass and inertia matrix.

   The kinematic chain is recursively constructed by adding children
   to each joint. (See CjrlJoint).

   The configuration of a robot is defined by a vector \f${\bf q}\f$ called the
   <b>configuration vector</b> and containing the values of each degree of
   freedom. The dimension of this vector is denoted by \f$n_{dof}\f$.

   The time derivative \f${\bf \dot{q}}\f$ of the configuration vector is called the <b>velocity vector</b>.

   The time derivative \f${\bf \ddot{q}}\f$ of the velocity vector. is called the <b>acceleration vector</b>.
*/

class CjrlDynamicRobot {
public:
  /**
     \name Kinematic chain
     @{
  */

  /**
     \brief Set the root joint of the robot.
  */
  virtual void rootJoint(CjrlJoint& inJoint) = 0;
  
  /**
     \brief Get the root joint of the robot.
  */
  virtual CjrlJoint* rootJoint() const = 0;

  /**
     \brief Get a vector containing all the joints.
  */
  virtual std::vector<CjrlJoint&> jointVector() = 0;

  /**
     @}
  */

  /** 
      \name Configuration, velocity and acceleration
  */
  
  /**
     \brief Set the current configuration of the robot.  

     \param inConfig the configuration vector \f${\bf q}\f$.
     
     \return true if success, false if failure (the dimension of the
     input vector does not fit the number of degrees of freedom of the
     robot).
  */
  virtual bool currentConfiguration(const ublas::vector& inConfig) = 0;

  /**
     \brief Get the current configuration of the robot.

     \return the configuration vector \f${\bf q}\f$.
  */
  virtual const ublas::vector& currentConfiguration() const = 0;

  /**
     \brief Set the current velocity of the robot.  

     \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.

     \return true if success, false if failure (the dimension of the
     input vector does not fit the number of degrees of freedom of the
     robot).
  */
  virtual bool currentVelocity(const ublas::vector& inVelocity) = 0;

  /**
     \brief Get the current velocity of the robot.

     \return the velocity vector \f${\bf \dot{q}}\f$.
  */
  virtual const ublas::vector& currentVelocity() const = 0;
  /**
     \brief Set the current acceleration of the robot.  

     \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.

     \return true if success, false if failure (the dimension of the
     input vector does not fit the number of degrees of freedom of the
     robot).
  */
  virtual bool currentAcceleration(const ublas::vector& inAcceleration) = 0;

  /**
     \brief Get the current acceleration of the robot.

     \return the acceleration vector \f${\bf \ddot{q}}\f$.
  */
  virtual const ublas::vector& currentAcceleration() const = 0;

 /**
     @}
  */

  /** 
      \name Forward kinematics and dynamics
  */
  
  /**
     \brief Compute forward kinematics.

     Update the position, velocity and accelerations of each
     joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.

  */
  virtual bool computeForwardKinematics() = 0;

  /**
     \brief Compute the dynamics of the center of mass.

     Compute the linear and  angular momentum and their time derivatives, at the center of mass.
  */
  virtual bool computeCenterOfMassDynamics() = 0;

  /**
     \brief Get the position of the center of mass.
  */
  virtual const ublas::vector& positionCenterOfMass() = 0;

  /**
     \brief Get the velocity of the center of mass.
  */
  virtual const ublas::vector& velocityCenterOfMass() = 0;

  /**
     \brief Get the acceleration of the center of mass.
  */
  virtual const ublas::vector& accelerationCenterOfMass() = 0;

  /**
     \brief Get the linear momentum of the robot.
  */
  virtual const ublas::vector& linearMomentumRobot() = 0;
  
  /**
     \brief Get the time-derivative of the linear momentum.
  */
  virtual const ublas::vector& derivativeLinearMomentum() = 0;

  /**
     \brief Get the angular momentum of the robot at the center of mass.
  */
  virtual const ublas::vector& angularMomentumRobot() = 0;
  
  /**
     \brief Get the time-derivative of the angular momentum at the center of mass.
  */
  virtual const ublas::vector& derivativeAngularMomentum() = 0;

  /**
     @}
  */

  /** 
      \name Jacobian fonctions
  */

  /**
     \brief Compute the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
  */
  virtual void computeJacobianCenterOfMass() = 0;

  /**
     \brief Get the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
  */
  virtual const ublas::matrix& jacobianCenterOfMass() const = 0;

  /**
     @}
  */
};


#endif
