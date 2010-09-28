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

#ifndef JRL_FOOT_H
#define JRL_FOOT_H

#include "robotDynamics/jrlJoint.h"
#include "robotDynamics/deprecated.h"


/**
   \brief This class represents a robot foot
 
   It is assumed that
   \li a foot is attached to the kinematic chain of the robot by a joint called \em ankle,
   \li contact between a foot and the ground is realized by a plane rectangular surface called the \em sole,
   \li the local frame of the foot is centered at the center of sole, x-axis pointing frontward, y-axis pointing leftward and z-axis pointing upward.

   \image html foot.png "A foot: the local frame is denoted by \f$(O, x, y, z)\f$. The center of the sole is denoted by C. The projection of O into the plane of the sole is denoted by H."
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
    \brief Get the ankle to which the foot is attached
    */
    virtual const CjrlJoint* associatedAnkle() const = 0;

    /**
    \brief Set the ankle to which the hand is attached.
    */
    virtual void  setAssociatedAnkle(const CjrlJoint* inJoint)=0;

    /** 
	\brief Get size of the rectagular sole

	\retval outLength length of the sole (see Figure)
	\retval outWidth width of the sole (see Figure)

    */
    virtual void getSoleSize(double &outLength, double &outWidth) const = 0;
    

    /** 
	\brief Set size of the rectagular sole

	\param inLength length of the sole (see Figure)
	\param inWidth width of the sole (see Figure)

    */
    virtual void 
      setSoleSize(const double &inLength, const double &inWidth) = 0;

    /**
       \brief  Get position of the ankle in the foot local coordinate frame

       \retval outCoordinates coordinates of the ankle joint center
    */
    virtual void 
      getAnklePositionInLocalFrame(vector3d& outCoordinates) const = 0;
    
    /**
       \brief  Set position of the ankle in the foot local coordinate frame

       \param inCoordinates coordinates of the ankle joint center
    */
    virtual void 
      setAnklePositionInLocalFrame(const vector3d& inCoordinates) = 0;

    /**
       \brief Get position of the sole center in foot local frame of the foot

       \retval outCoordinates coordinates of the center C of the sole 

       \deprecated Should be always 0.
    */
    JRLDEPRECATED(virtual void getSoleCenterInLocalFrame
		  (vector3d& outCoordinates) const) = 0;

    /**
       \brief Set position of the sole center in foot local frame of the foot

       \param inCoordinates coordinates of the center C of the sole 
       (see Figure) 
    */
    virtual void setSoleCenterInLocalFrame(const vector3d& inCoordinates) = 0;

    /**
       \brief Get position of projection of center of local frame in sole plane

       \retval outCoordinates coordinates of the projection H of the center 
       of the local frame in the sole plane (see Figure) 

       \deprecated Should be always 0
    */
    JRLDEPRECATED(virtual void 
		  getProjectionCenterLocalFrameInSole
		  (vector3d& outCoordinates) const) = 0;

    /**
       \brief Set position of projection of center of local frame in sole plane

       \param inCoordinates coordinates of the projection H of the center 
       of the local frame in the sole plane (see Figure) 
    */
    virtual void 
      setProjectionCenterLocalFrameInSole(const vector3d& inCoordinates) = 0;

};


#endif
