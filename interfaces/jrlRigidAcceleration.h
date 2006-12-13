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

#ifndef JRL_RIGID_ACCELERATION_H
#define JRL_RIGID_ACCELERATION_H

/** 
    \brief This class represents the acceleration of a rigid body.

    The acceleration is represented by
    \li a linear acceleration vector (time derivative of the linear velocity vector) and
    \li a rotation acceleration vector (the time derivative of the rotation velocity vector).
*/

class CjrlRigidAcceleration {
public:
  /**
     \brief Constructor
  */
  CjrlRigidAcceleration(const vector3d& inLinearAcceleration, const vector3d& inRotationAcceleration):
    attLinearAcceleration(inLinearAcceleration), attRotationAcceleration(inRotationAcceleration) {};

  /**
     Get the linear acceleration vector.
  */
  vector3d linearAcceleration() {return attLinearAcceleration;};

  /**
     Set the linear acceleration vector.
  */
  linearAcceleration(vector3d inLinearAcceleration) {attLinearAcceleration = inLinearAcceleration;};

  /**
     Get the rotation acceleration vector.
  */
  vector3d rotationAcceleration() {return attRotationAcceleration;};

  /**
     Set the rotation acceleration vector.
  */
  rotationAcceleration(vector3d inRotationAcceleration) {attRotationAcceleration = inRotationAcceleration;};

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
