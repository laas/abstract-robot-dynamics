// Copyright 2006, 2007, 2008, 2009, 2010, Florent Lamiraux, CNRS/AIST
//
// This file is part of abstract-robot-dynamics.
// abstract-robot-dynamics is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// abstract-robot-dynamics is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// abstract-robot-dynamics.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef ABSTRACT_ROBOT_DYNAMICS_ROBOT_DYNAMICS_OBJECT_CONSTRUCTOR
# define ABSTRACT_ROBOT_DYNAMICS_ROBOT_DYNAMICS_OBJECT_CONSTRUCTOR
# include <abstract-robot-dynamics/fwd.hh>
# include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>

/// \brief The creation of an object.
class CjrlRobotDynamicsObjectFactory
{
public:
  /// \brief Destructor.
  virtual ~CjrlRobotDynamicsObjectFactory() {}

  /// \brief Construct and return a pointer to a dynamic robot.
  virtual JrlDynamicRobotPtr createDynamicRobot() = 0;

  /// \brief Construct and return a pointer to a humanoid dynamic robot.
  virtual JrlHumanoidDynamicRobotPtr
  createHumanoidDynamicRobot() = 0;

  /// \brief Construct and return a pointer to a freeflyer joint.

  /// \param inInitialPosition position of the local frame of the
  /// joint when the robot is in initial configuration.
  virtual JrlJointPtr
  createJointFreeflyer(const matrix4d& inInitialPosition) = 0;

  /// \brief Construct and return a pointer to a rotation joint.
  ///
  /// \param inInitialPosition position of the local frame of the
  /// joint when the robot is in initial configuration.
  virtual JrlJointPtr
  createJointRotation(const matrix4d& inInitialPosition) = 0;

  /// \brief Construct and return a pointer to a translation joint.
  ///
  /// \param inInitialPosition position of the local frame of the
  /// joint when the robot is in initial configuration.
  virtual JrlJointPtr
  createJointTranslation(const matrix4d& inInitialPosition) = 0;

  /// \brief Construct and return a pointer to a body
  virtual JrlBodyPtr createBody() = 0;

  /// \brief Construct and return a pointer to a hand.
  ///
  /// \param inWristJoint The joint the hand is attached to.
  virtual JrlHandPtr
  createHand(JrlJointPtr inWristJoint) = 0;

  /// \brief Construct and return a pointer to a foot.
  ///
  /// \param inAnkle The joint the foot is attached to.
  virtual JrlFootPtr
  createFoot(JrlJointPtr inAnkle) = 0;
};

#endif //! ABSTRACT_ROBOT_DYNAMICS_ROBOT_DYNAMICS_OBJECT_CONSTRUCTOR
