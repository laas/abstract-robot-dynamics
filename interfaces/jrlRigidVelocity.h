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

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"

/** 
    \brief This class represents the velocity of a rigid body.

    The velocity is represented by
    \li a linear velocity vector \f${\bf v}\f$ and
    \li a rotation velocity vector \f${\bf \omega}\f$.
*/

class CjrlRigidVelocity {
public:
  /**
     \brief Constructor
  */
  CjrlRigidVelocity() {};
  /**
     \brief Constructor
  */
  CjrlRigidVelocity(const vector3d& inLinearVelocity, const vector3d& inRotationVelocity) {
    attLinearVelocity = inLinearVelocity; 
    attRotationVelocity = inRotationVelocity;
  };


  /**
     Get the linear velocity vector.
  */
  const vector3d& linearVelocity() const {return attLinearVelocity;}

  /**
     Set the linear velocity vector.
  */
  void linearVelocity(const vector3d& inLinearVelocity) {attLinearVelocity = inLinearVelocity;}

  /**
     Get the rotation velocity vector.
  */
  const vector3d& rotationVelocity() const {return attRotationVelocity;}

  /**
     Set the rotation velocity vector.
  */
  void rotationVelocity(const vector3d& inRotationVelocity) {attRotationVelocity = inRotationVelocity;}

private:
  /**
     \brief Linear velocity vector.
  */
  vector3d attLinearVelocity;
  /**
     \brief Angular velocity vector.
  */
  vector3d attRotationVelocity;
};

#endif
