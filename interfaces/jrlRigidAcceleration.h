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

template <class V3>
class CjrlRigidAcceleration {
public:
  /**
     \brief Constructor
  */
  CjrlRigidAcceleration<V3>(const V3& inLinearAcceleration, const V3& inRotationAcceleration):
    attLinearAcceleration(inLinearAcceleration), attRotationAcceleration(inRotationAcceleration) {}

  /**
     Get the linear acceleration vector.
  */
  V3 linearAcceleration() const {return attLinearAcceleration;};

  /**
     Set the linear acceleration vector.
  */
  void linearAcceleration(const V3& inLinearAcceleration) {attLinearAcceleration = inLinearAcceleration;}

  /**
     Get the rotation acceleration vector.
  */
  V3 rotationAcceleration() const {return attRotationAcceleration;}

  /**
     Set the rotation acceleration vector.
  */
  void rotationAcceleration(const V3& inRotationAcceleration) {attRotationAcceleration = inRotationAcceleration;}

private:
  /**
     \brief Linear acceleration vector.
  */
  V3 attLinearAcceleration;
  /**
     \brief Angular acceleration vector.
  */
  V3 attRotationAcceleration;
};

#endif
