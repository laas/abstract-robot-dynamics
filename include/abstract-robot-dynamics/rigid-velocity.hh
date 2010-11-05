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

#ifndef ABSTRACT_ROBOT_DYNAMICS_RIGID_VELOCITY_HH
# define ABSTRACT_ROBOT_DYNAMICS_RIGID_VELOCITY_HH
# include <abstract-robot-dynamics/fwd.hh>

/// \brief This class represents the velocity of a rigid body.
///
/// The velocity is represented by
/// \li a linear velocity vector \f${\bf v}\f$ and
/// \li a rotation velocity vector \f${\bf \omega}\f$.
class CjrlRigidVelocity
{
public:
  /// \brief Constructor.
  CjrlRigidVelocity() {}

  /// \brief Constructor.
  CjrlRigidVelocity(const vector3d& inLinearVelocity,
		    const vector3d& inRotationVelocity)
    : attLinearVelocity (inLinearVelocity),
      attRotationVelocity (inRotationVelocity)
  {}


  /// \brief Get the linear velocity vector.
  const vector3d& linearVelocity() const
  {
    return attLinearVelocity;
  }

  /// \brief Set the linear velocity vector.
  void linearVelocity(const vector3d& inLinearVelocity)
  {
    attLinearVelocity = inLinearVelocity;
  }

  /// \brief Get the rotation velocity vector.
  const vector3d& rotationVelocity() const
  {
    return attRotationVelocity;
  }

  /// \brief Set the rotation velocity vector.
  void rotationVelocity(const vector3d& inRotationVelocity)
  {
    attRotationVelocity = inRotationVelocity;
  }

private:
  /// \brief Linear velocity vector.
  vector3d attLinearVelocity;
  /// \brief Angular velocity vector.
  vector3d attRotationVelocity;
};

#endif //! ABSTRACT_ROBOT_DYNAMICS_RIGID_VELOCITY_HH
