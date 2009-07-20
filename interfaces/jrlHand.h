#ifndef JRL_HAND_H
#define JRL_HAND_H

#include <vector>
#include "deprecated.h"
#include "jrlJoint.h"


/**
   \brief This class represents a robot hand.
   A hand has a central point referenced in the wrist joint frame and three axis
 */

class CjrlHand
{
public:

    /**
    \brief Destructor
     */
    virtual ~CjrlHand()
    {};

    /**
    \brief Get the wrist joint to which the hand is attached
    */
    virtual const CjrlJoint* associatedWrist() = 0;

    /**
    \brief Get the wrist joint to which the hand is attached
    */
    virtual void setAssociatedWrist(const CjrlJoint * inJoint ) = 0;

  /**
     \brief Get the center of the hand

     \retval outCenter Center of the hand in the frame of the wrist.

  */
  virtual void getCenter(vector3d& outCenter) const = 0;

  /**
     \brief Set the center of the hand

     \param inCenter Center of the hand in the frame of the wrist.

  */
  virtual void setCenter(const vector3d& inCenter) = 0;

  /**
     \brief Get thumb axis when had is in open position

     \retval outThumbAxis Axis of the thumb in wrist frame in open position

  */
  virtual void getThumbAxis(vector3d& outThumbAxis) const = 0;

  /**
     \brief Set thumb axis in wrist frame when had is in open position

     \param inThumbAxis Axis of the thumb in wrist frame in open position
  */
  virtual void setThumbAxis(const vector3d& inThumbAxis) = 0;

  /**
     \brief Get forefinger axis

     \retval outForeFingerAxis axis of the forefinger in wrist frame 
     in open position 
  */
  virtual void getForeFingerAxis(vector3d& outForeFingerAxis) const = 0;

  /**
     \brief Set forefinger axis

     \param inForeFingerAxis axis of the forefinger in wrist frame 
     in open position 
  */
  virtual void setForeFingerAxis(const vector3d& inForeFingerAxis) = 0;

  /**
     \brief Get palm normal

     \retval outPalmNormal normal to the palm in the frame of the wrist.
  */
  virtual void getPalmNormal(vector3d& outPalmNormal) const = 0;

  /**
     \brief Set palm normal

     \param inPalmNormal normal to the palm in the frame of the wrist.
  */
  virtual void setPalmNormal(const vector3d& inPalmNormal) = 0;

    /**
       \name Deprecated methods
       @{
    */

    /**
    \brief Get the center of the hand in the wrist frame
     */
    virtual JRLDEPRECATED( vector3d& centerInWristFrame() ) = 0;

  
    /**
    \brief Get the axis defined by the thumb being held up in the way an "okay" sign is made. The returned axis is a 3d vector in the wrist frame.
     */
    virtual JRLDEPRECATED( vector3d& okayAxisInWristFrame() ) = 0;

    /**
    \brief Get the axis defined by the forefinger being. The returned axis is a 3d vector in the wrist frame,
     */
    virtual JRLDEPRECATED( vector3d& showingAxisInWristFrame() ) = 0;

    /**
    \brief Get the axis orthogonal to the palm. The returned axis is a 3d vector in the wrist frame pointing to the direction where all fingers can join,.
     */
    virtual JRLDEPRECATED( vector3d& palmAxisInWristFrame() ) = 0;

  /**
     @}
  */
};



#endif
