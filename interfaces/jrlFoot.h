/*
 *   Copyright (c) 2006, 2007, 2008, 2009 AIST-CNRS 
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Florent Lamiraux CNRS
 *
 */

#ifndef JRL_FOOT_H
#define JRL_FOOT_H

#include "jrlJoint.h"


/**
   \brief This class represents a robot foot
 
   It is assumed that
   \li a foot is attached to the kinematic chain of the robot by a joint called \em ankle,
   \li contact between a foot and the ground is realized by a plane rectangular surface called the \em sole.

   \image html foot.png "A foot: the local frame is denoted by \f(O, x, y, z)\f. The center of the sole is denoted by C. The projection of O into the plane of the sole is denoted by H.
*/

class CjrlFoot
{
public:

    /**
    \brief Destructor
     */
    virtual ~CjrlFoot()
    {};

    /**
    \brief Get the wrist joint to which the hand is attached
    */
    virtual CjrlJoint* associatedAnkle() = 0;

    /** 
	\brief Get size of the rectagular sole

	\retval outLength length of the sole (see Figure)
	\retval outWidth width of the sole (see Figure)

    */
    virtual void soleSize(double &outLength, double &outWidth)=0;
    
    /**
       \brief  Get position of the ankle in the foot local coordinate frame

       \retval outCoordinates coordinates of the ankle joint center
    */
    virtual void anklePositionInLocalFrame(vector3d& outCoordinates)=0;

    /**
       \brief Get position of the sole center in foot local frame of the foot

       \refval outCoordinates coordinates of the center C of the sole (see Figure) 
    */
    virtual void soleCenterInLocalFrame(vector3d& outCoordinates)=0;

    /**
       \brief Get position of the projection of center of local frame in sole plane

       \refval outCoordinates coordinates of the projection H of the center of the local frame in the sole plane (see Figure) 
    */
    virtual void projectionCenterLocalFrameInSole(vector3d& outCoordinates)=0;

};


#endif
