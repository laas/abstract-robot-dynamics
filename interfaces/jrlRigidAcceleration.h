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
