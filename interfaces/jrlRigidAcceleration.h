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

#ifndef JRL_RIGID_ACCELERATION_H
#define JRL_RIGID_ACCELERATION_H


/**
    \brief This class represents the acceleration of a rigid body.

    The acceleration is represented by
    \li a linear acceleration vector \f${\bf \dot{v}}\f$ (time derivative of the linear velocity vector) and
    \li a rotation acceleration vector \f${\bf \dot{\omega}}\f$ (the time derivative of the rotation velocity vector).
*/

class CjrlRigidAcceleration {
public:
  /**
     \brief Constructor
  */
  CjrlRigidAcceleration(const vector3d& inLinearAcceleration, const vector3d& inRotationAcceleration) {
    attLinearAcceleration = inLinearAcceleration;
    attRotationAcceleration = inRotationAcceleration;
  };


  /**
     Get the linear acceleration vector.
  */
  const vector3d& linearAcceleration() const {return attLinearAcceleration;};

  /**
     Set the linear acceleration vector.
  .3
*/
  void linearAcceleration(const vector3d& inLinearAcceleration) {attLinearAcceleration = inLinearAcceleration;}

  /**
     Get the rotation acceleration vector.
  */
  const vector3d& rotationAcceleration() const {return attRotationAcceleration;}

  /**
     Set the rotation acceleration vector.
  */
  void rotationAcceleration(const vector3d& inRotationAcceleration) {attRotationAcceleration = inRotationAcceleration;}

private:
  /**
     \brief Linear acceleration vector.
  */
  vector3d attLinearAcceleration;
  /**
     \brief Angular acceleration vector.
  */
  vector3d attRotationAcceleration;
};

#endif
