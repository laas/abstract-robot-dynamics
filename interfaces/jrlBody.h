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

#ifndef JRL_BODY_H
#define JRL_BODY_H

class CjrlBody {
  /**
     \brief Get position of center of mass in joint local reference frame.
  */
  virtual const vector3d& localCenterOfMass() const = 0;

  /**
     \brief Set postion of center of mass in joint reference frame.
  */
  virtual void localCenterOfMass(const vector3d& inlocalCenterOfMass) = 0;

  /**
     \brief Get Intertia matrix expressed in joint local reference frame.
  */
  virtual const matrix3d& inertiaMatrix() const = 0;

  /**
     \brief Set inertia matrix.
  */
  virtual void inertiaMatrix(const matrix3d& inInertiaMatrix) = 0;

  /**
     \brief Get const pointer to the joint the body is attached to.
  */
  virtual const CjrlJoint* joint() = 0 const;
};


#endif
