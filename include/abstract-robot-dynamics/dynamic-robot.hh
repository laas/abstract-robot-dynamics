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

#ifndef ABSTRACT_ROBOT_DYNAMICS_ROBOT_HH
# define ABSTRACT_ROBOT_DYNAMICS_ROBOT_HH
# include <abstract-robot-dynamics/fwd.hh>
# include <abstract-robot-dynamics/joint.hh>

/**
   \brief Abstract class that instantiates a robot with dynamic properties.

   \par Definition
   A robot is a kinematic chain of joints. To each joint is attached a body.
   Each body has a mass and inertia matrix.
   The kinematic chain is recursively constructed by adding children
   to each joint. (See CjrlJoint).
   The configuration of a robot is defined by a vector \f${\bf q}\f$ called the
   <b>configuration vector</b> and containing the values of each degree of
   freedom. The dimension of this vector is denoted by \f$n_{dof}\f$.

   The time derivative \f${\bf \dot{q}}\f$ of the configuration vector
   is called the <b>velocity vector</b>.

   The time derivative \f${\bf \ddot{q}}\f$ of the velocity vector. is
   called the <b>acceleration vector</b>.

   \par Control of the implementation
   In some cases, it is desirable to control the implementation of this
   class in order to selectively compute only some kinematic and dynamic
   values. For instance, for speed of computation purposes, one may want
   to compute only velocities and not accelerations. In order to keep the
   interface light, a control mechanism based on properties is proposed
   through the following methods: isSupported(), getProperty(), setProperty().
   Each implementation is responsible for its own methods. However, in order to
   keep some compatibility, some recommended methods are listed in
   \ref abstractRobotDynamics_commands "this page".

   \par Actuated Joints.
   In order to make a distinction between actuated joints and none
   actuacted joints,
   a vector of actuated joints is provided through method:  getActuatedJoints().
*/
class CjrlDynamicRobot
{
public:
  /// \name Initialization
  /// \{

  /// \brief Initialize data-structure necessary to dynamic computations
  /// This function should be called after building the tree of joints.
  virtual bool initialize() = 0;

  /// \brief Destructor
  virtual ~CjrlDynamicRobot() {}

  /// \}

  /// \name Kinematic chain
  /// \{

  /// \brief Set the root joint of the robot.
  virtual void rootJoint(CjrlJoint& inJoint) = 0;

  /// \brief Get the root joint of the robot.
  virtual to_pointer<CjrlJoint>::type rootJoint() const = 0;

  /// \brief Get a vector containing all the joints.
  virtual std::vector<to_pointer<CjrlJoint>::type > jointVector() = 0;

  /// \brief Get the chain of joints between two joints
  /// \param inStartJoint First joint.
  /// \param inEndJoint Second joint.
  virtual std::vector<to_pointer<CjrlJoint>::type >
  jointsBetween(const CjrlJoint& inStartJoint,
		const CjrlJoint& inEndJoint) const = 0;

  /// \brief Get the upper bound for ith dof.
  virtual double upperBoundDof(unsigned int inRankInConfiguration) = 0;

  /// \brief Get the lower bound for ith dof.
  virtual double lowerBoundDof(unsigned int inRankInConfiguration) = 0;

  /// \brief Compute the upper bound for ith dof using other
  /// configuration values if possible.
  virtual double upperBoundDof(unsigned int inRankInConfiguration,
			       const vectorN& inConfig) = 0;

  /// \brief Compute the lower bound for ith dof using other
  /// configuration values if possible.
  virtual double lowerBoundDof(unsigned int inRankInConfiguration,
			       const vectorN& inConfig) = 0;

  /// \brief Get the number of degrees of freedom of the robot.
  virtual unsigned int numberDof() const = 0;

  /// \brief Set the joint ordering in the configuration vector
  ///
  /// \param inJointVector Vector of the robot joints
  ///
  /// Specifies the order of the joints in the configuration vector.
  /// The vector should contain all the joints of the current robot.
  virtual void
  setJointOrderInConfig
  (std::vector<to_pointer<CjrlJoint>::type > inJointVector) = 0;

  /// \}

  /// \name Configuration, velocity and acceleration

  /// \brief Set the current configuration of the robot.
  ///
  /// \param inConfig the configuration vector \f${\bf q}\f$.
  ///
  /// \return true if success, false if failure (the dimension of the
  /// input vector does not fit the number of degrees of freedom of the
  /// robot).
  virtual bool currentConfiguration(const vectorN& inConfig) = 0;

  /// \brief Get the current configuration of the robot.
  ///
  /// \return the configuration vector \f${\bf q}\f$.
  virtual const vectorN& currentConfiguration() const = 0;

  /// \brief Set the current velocity of the robot.
  ///
  /// \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.
  ///
  /// \return true if success, false if failure (the dimension of the
  /// input vector does not fit the number of degrees of freedom of the
  /// robot).
  virtual bool currentVelocity(const vectorN& inVelocity) = 0;

  /// \brief Get the current velocity of the robot.
  ///
  /// \return the velocity vector \f${\bf \dot{q}}\f$.
  virtual const vectorN& currentVelocity() const = 0;

  /// \brief Set the current acceleration of the robot.
  ///
  /// \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.
  ///
  /// \return true if success, false if failure (the dimension of the
  /// input vector does not fit the number of degrees of freedom of the
  /// robot).
  virtual bool currentAcceleration(const vectorN& inAcceleration) = 0;

  /// \brief Get the current acceleration of the robot.
  ///
  /// \return the acceleration vector \f${\bf \ddot{q}}\f$.
  virtual const vectorN& currentAcceleration() const = 0;

  /// \brief Get the current forces of the robot.
  ///
  /// \return the force vector \f${\bf f}\f$.
  virtual const matrixNxP& currentForces() const = 0;

  /// \brief Get the current torques of the robot.
  ///
  /// \return the torque vector \f${\bf \tau }\f$.
  virtual const matrixNxP& currentTorques() const = 0;

  /// \brief Get the current joint torques of the robot.
  ///
  /// The torques are computed by internal calls to the
  /// direct dynamic computations. Dynamics is computed in free-floating
  /// mode, supposing no contact with the environments, and knowing
  /// given position, velocity and acceleration. This accessor only give
  /// a reference on the already-computed values.
  /// \return the torque vector \f${\bf \tau }\f$.
  virtual const vectorN& currentJointTorques() const = 0;

  /// \}

  /// \name Forward kinematics and dynamics

  /// \brief Compute forward kinematics.
  ///
  /// Update the position, velocity and accelerations of each
  /// joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.
  virtual bool computeForwardKinematics() = 0;

  /// \brief Compute the dynamics of the center of mass.
  ///
  /// Compute the linear and angular momentum and their time
  /// derivatives, at the center of mass.
  virtual bool computeCenterOfMassDynamics() = 0;

  /// \brief Get the position of the center of mass.
  virtual const vector3d& positionCenterOfMass() const = 0;

  /// \brief Get the velocity of the center of mass.
  virtual const vector3d& velocityCenterOfMass() = 0;

  /// \brief Get the acceleration of the center of mass.
  virtual const vector3d& accelerationCenterOfMass() = 0;

  /// \brief Get the linear momentum of the robot.
  virtual const vector3d& linearMomentumRobot() = 0;

  /// \brief Get the time-derivative of the linear momentum.
  virtual const vector3d& derivativeLinearMomentum() = 0;

  /// \brief Get the angular momentum of the robot at the center of mass.
  virtual const vector3d& angularMomentumRobot() = 0;

  /// \brief Get the time-derivative of the angular momentum at the
  /// center of mass.
  virtual const vector3d& derivativeAngularMomentum() = 0;

  /// \brief Get the total mass of the robot
  virtual double mass() const =0;

  /// \}

  /// \name Control of the implementation
  /// \{

  /// \brief Whether the specified property in implemented.
  virtual bool isSupported(const std::string &);

  /// \brief Get property corresponding to command name.
  ///
  /// \param inProperty name of the property.
  /// \retval outValue value of the property if implemented.
  ///
  /// \note The returned string needs to be cast into the right type
  /// (double, int,...).
  virtual bool getProperty(const std::string &,
			   std::string& ) const;

  /// \brief Set property corresponding to command name.
  ///
  /// \param inProperty name of the property.
  /// \param inValue value of the property.
  ///
  /// \note The value string is obtained by writing the
  /// corresponding value in a string (operator<<).
  virtual bool setProperty(std::string &,
			   const std::string& );

  /// \}

  /**
     \brief Compute and get position and orientation jacobian

     \param inStartJoint First joint in the chain of joints influencing
     the jacobian.
     \param inEndJoint Joint where the control frame is located.
     \param inFrameLocalPosition Position of the control frame in inEndJoint
     local frame.
     \retval outjacobian computed jacobian matrix.
     \param offset is the rank of the column from where the jacobian is written.
     \param inIncludeStartFreeFlyer Option to include the contribution of a
     fictive freeflyer superposed with inStartJoint

     \return false if matrix has inadequate size. Number of columns
     in matrix outJacobian must be at least numberDof() if
     inIncludeStartFreeFlyer = true.
     It must be at least numberDof()-6 otherwise.
  */
  virtual bool getJacobian(const CjrlJoint& inStartJoint,
			   const CjrlJoint& inEndJoint,
			   const vector3d& inFrameLocalPosition,
			   matrixNxP& outjacobian,
			   unsigned int offset = 0,
			   bool inIncludeStartFreeFlyer = true) = 0;

  virtual bool getPositionJacobian(const CjrlJoint& inStartJoint,
				   const CjrlJoint& inEndJoint,
				   const vector3d& inFrameLocalPosition,
				   matrixNxP& outjacobian,
				   unsigned int offset = 0,
				   bool inIncludeStartFreeFlyer = true) = 0;

  virtual bool getOrientationJacobian(const CjrlJoint& inStartJoint,
				      const CjrlJoint& inEndJoint,
				      matrixNxP& outjacobian,
				      unsigned int offset = 0,
				      bool inIncludeStartFreeFlyer = true) = 0;

  virtual bool getJacobianCenterOfMass(const CjrlJoint& inStartJoint,
				       matrixNxP& outjacobian,
				       unsigned int offset = 0,
				       bool inIncludeStartFreeFlyer = true) = 0;

  ///\name Inertia matrix related methods
  /// \{

  /// \brief Compute the inertia matrix of the robot according wrt
  /// \f${\bf q}\f$.
  virtual void computeInertiaMatrix() = 0;

  /// \brief Get the inertia matrix of the robot according wrt \f${\bf q}\f$.
  virtual const matrixNxP& inertiaMatrix() const = 0;
  /// \}

  /// \name Actuated joints related methods.
  /// \{

  /// \brief Returns the list of actuated joints.
  virtual const std::vector<to_pointer<CjrlJoint>::type >&
  getActuatedJoints() const = 0;

  /// \brief Specifies the list of actuated joints.
  virtual void
  setActuatedJoints
  (std::vector<to_pointer<CjrlJoint>::type >& lActuatedJoints) = 0;

  /// \}


  /*! \brief Compute Speciliazed InverseKinematics between two joints.

    Specialized means that this method can be re implemented to be
    extremly efficient and used the particularity of your robot.
    For instance in some case, it is possible to use an exact inverse
    kinematics to compute a set of articular value.

    This method does not intend to replace an architecture computing
    inverse kinematics through the Jacobian.

    jointRootPosition and jointEndPosition have to be expressed in the same
    frame.

    \param[in] jointRoot: The root of the joint chain for which the specialized
    inverse kinematics should be computed.

    \param[in] jointEnd: The end of the joint chain for which the specialized
    inverse kinematics should be computed.

    \param[in] jointRootPosition: The desired position of the root.

    \param[in] jointEndPosition: The end position of the root.

    \param[out] q: Result i.e. the articular values.
  */
  virtual bool
  getSpecializedInverseKinematics(const CjrlJoint & ,
				  const CjrlJoint & ,
				  const matrix4d & ,
				  const matrix4d & ,
				  vectorN &);
  /// \}
};


// Separate instantiation is required to avoid warnings with g++.
// Unused parameters in functions/methods triggers a warning unless
// they have no name. But in this, Doxygen is not able to document the
// interface in an efficient way. A good compromise is class
// declaration with no implementation and parameters with name, then
// implementation where unused parameters have no name.
inline bool
CjrlDynamicRobot::getProperty(const std::string&, std::string&) const
{
  return false;
}

inline bool
CjrlDynamicRobot::setProperty(std::string&, const std::string&)
{
  return false;
}

inline bool
CjrlDynamicRobot::isSupported(const std::string&)
{
  return false;
}

inline bool
CjrlDynamicRobot::getSpecializedInverseKinematics(const CjrlJoint&,
						  const CjrlJoint&,
						  const matrix4d&,
						  const matrix4d&,
						  vectorN&)
{
  return false;
}

#endif //! ABSTRACT_ROBOT_DYNAMICS_ROBOT_HH
