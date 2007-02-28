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

#ifndef JRL_BODY_H
#define JRL_BODY_H

template <class Mnxp, class M4x4, class M3x3, class Vn, class V3> class CjrlJoint;

template <class Mnxp, class M4x4, class M3x3, class Vn, class V3>
class CjrlBody
{
 public:

    /**
       \brief Get position of center of mass in joint local reference frame.
    */
    virtual const V3& localCenterOfMass() const = 0;

    /**
       \brief Set postion of center of mass in joint reference frame.
    */
    virtual void localCenterOfMass(const V3& inlocalCenterOfMass) = 0;

    /**
       \brief Get Intertia matrix expressed in joint local reference frame.
    */
    virtual const M3x3& inertiaMatrix() const = 0;

    /**
       \brief Set inertia matrix.
    */
    virtual void inertiaMatrix(const M3x3& inInertiaMatrix) = 0;
    
    /**
    \brief Get mass.
     */
    virtual double mass() const = 0;

    /**
    \brief Set mass.
     */
    virtual void mass(double inMass) = 0;

    /**
       \brief Get const pointer to the joint the body is attached to.
    */
    virtual const CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* joint() const = 0 ;
};


#endif
