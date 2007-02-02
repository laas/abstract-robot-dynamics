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

#ifndef JRL_RIGID_VELOCITY_H
#define JRL_RIGID_VELOCITY_H

/** 
    \brief This class represents the velocity of a rigid body.

    The velocity is represented by
    \li a linear velocity vector \f${\bf v}\f$ and
    \li a rotation velocity vector \f${\bf \omega}\f$.
*/

template <class V3> class CjrlRigidVelocity {
public:
  /**
     \brief Constructor
  */
  CjrlRigidVelocity<V3>() {};
  /**
     \brief Constructor
  */
  CjrlRigidVelocity<V3>(const V3& inLinearVelocity, const V3& inRotationVelocity):
    attLinearVelocity(inLinearVelocity), attRotationVelocity(inRotationVelocity) {};

  /**
     Get the linear velocity vector.
  */
  V3 linearVelocity() const {return attLinearVelocity;};

  /**
     Set the linear velocity vector.
  */
  linearVelocity(const V3& inLinearVelocity) {attLinearVelocity = inLinearVelocity;};

  /**
     Get the rotation velocity vector.
  */
  V3 rotationVelocity() const {return attRotationVelocity;};

  /**
     Set the rotation velocity vector.
  */
  rotationVelocity(const V3& inRotationVelocity) {attRotationVelocity = inRotationVelocity;};

private:
  /**
     \brief Linear velocity vector.
  */
  V3 attLinearVelocity;
  /**
     \brief Angular velocity vector.
  */
  V3 attRotationVelocity;
};

#endif
