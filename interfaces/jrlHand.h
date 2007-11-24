#ifndef JRL_HAND_H
#define JRL_HAND_H

#include <vector>
#include "jrlJoint.h"


/**
   \brief This class represents a robot hand.
   A hand has a central point referenced in the wrist joint frame and three axis
 */

class CjrlHand
{
public:

    /**
    \brief Get the wrist joint to which the hand is attached
    */
    virtual CjrlJoint* associatedWrist() = 0;

    /**
    \brief Get the center of the hand in the wrist frame
     */
    virtual vector3d& centerInWristFrame() = 0;

    /**
    \brief Get the axis defined by the thumb being held up in the way an "okay" sign is made. The returned axis is a 3d vector in the wrist frame.
     */
    virtual vector3d& okayAxisInWristFrame() = 0;

    /**
    \brief Get the axis defined by the forefinger being. The returned axis is a 3d vector in the wrist frame,
     */
    virtual vector3d& showingAxisInWristFrame() = 0;

    /**
    \brief Get the axis orthogonal to the palm. The returned axis is a 3d vector in the wrist frame pointing to the direction where all fingers can join,.
     */
    virtual vector3d& palmAxisInWristFrame() = 0;

};


#endif
