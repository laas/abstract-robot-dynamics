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

#ifndef ABSTRACT_ROBOT_DYNAMICS_BODY_HH
# define ABSTRACT_ROBOT_DYNAMICS_BODY_HH
# include <abstract-robot-dynamics/fwd.hh>

class CjrlBody
{
public:
  /// \brief Get position of center of mass in joint local reference frame.
  virtual const vector3d& localCenterOfMass() const = 0;

  /// \brief Set postion of center of mass in joint reference frame.
  virtual void localCenterOfMass(const vector3d& inlocalCenterOfMass) = 0;

  /// \brief Get Intertia matrix expressed in joint local reference frame.
  virtual const matrix3d& inertiaMatrix() const = 0;

  /// \brief Set inertia matrix.
  virtual void inertiaMatrix(const matrix3d& inInertiaMatrix) = 0;

  /// \brief Get mass.
  virtual double mass() const = 0;

  /// \brief Set mass.
  virtual void mass(double inMass) = 0;

  /// \brief Get const pointer to the joint the body is attached to.
  virtual JrlJointConstPtr joint() const = 0;

  /// \brief Destructor
  virtual ~CjrlBody() {}
};

#endif //! ABSTRACT_ROBOT_DYNAMICS_BODY_HH
