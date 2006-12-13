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

#ifndef JRL_RIGID_VELOCITY_H
#define JRL_RIGID_VELOCITY_H

/** 
    \brief This class represents the velocity of a rigid body.

    The velocity is represented by
    \li a linear velocity vector and
    \li a rotation velocity vector.
*/

class CjrlRigidVelocity {
public:
  /**
     \brief Constructor
  */
  CjrlRigidVelocity(const vector3d& inLinearVelocity, const vector3d& inRotationVelocity):
    attLinearVelocity(inLinearVelocity), attRotationVelocity(inRotationVelocity) {};

  /**
     Get the linear velocity vector.
  */
  vector3d linearVelocity() {return attLinearVelocity;};

  /**
     Set the linear velocity vector.
  */
  linearVelocity(vector3d inLinearVelocity) {attLinearVelocity = inLinearVelocity;};

  /**
     Get the rotation velocity vector.
  */
  vector3d rotationVelocity() {return attRotationVelocity;};

  /**
     Set the rotation velocity vector.
  */
  rotationVelocity(vector3d inRotationVelocity) {attRotationVelocity = inRotationVelocity;};

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
