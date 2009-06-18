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

#ifndef JRL_HUMANOID_DYNAMIC_ROBOT
#define JRL_HUMANOID_DYNAMIC_ROBOT

#include "jrlDynamicRobot.h"
#include "jrlHand.h"


/**
    \brief Abstract class that instantiate a humanoid robot with dynamics.
 
    This class derives for CjrlDynamicRobot and instantiate properties specific to humanoid robots. 
    \li it provides pointers to the feet and hand joints,
    \li it provides pointers to the joint corresponding to the gaze,
    \li it computes the Zero Momentum Point.
*/

class CjrlHumanoidDynamicRobot : public virtual CjrlDynamicRobot
{
public:

    /**
    \brief Destructor
     */
    virtual ~CjrlHumanoidDynamicRobot()
    {};

    /**
       \name Joints specific to humanoid robots
    */

    /**
       \brief Set the pointer to the waist.
    */
    virtual void waist(CjrlJoint* inWaist) = 0;

    /**
        \brief Get a pointer to the waist.
    */
    virtual CjrlJoint* waist() = 0;

    /**
       \brief Set the pointer to the chest.

       \note for some humanoid robots, the waist and the chest are the same joints.
    */
    virtual void chest(CjrlJoint* inChest) = 0;

    /**
        \brief Get a pointer to the chest.

       \note for some humanoid robots, the waist and the chest are the same joints.
    */
    virtual CjrlJoint* chest() = 0;

    /**
      \brief Set the pointer to the left wrist joint.
    */
    virtual void leftWrist(CjrlJoint* inLefWrist) = 0;

    /**
      \brief Get a pointer to the left wrist.
    */
    virtual CjrlJoint* leftWrist() = 0;

    /**
      \brief Set the pointer to the right wrist joint.
    */
    virtual void rightWrist(CjrlJoint* inRightWrist) = 0;

    /**
      \brief Get a pointer to the right wrist.
    */
    virtual CjrlJoint* rightWrist() = 0;

    /**
      \brief Set the pointer to the right hand
    */
    virtual void rightHand(CjrlHand* inRightHand) = 0;

    /**
      \brief Get a pointer to the right hand
    */
    virtual CjrlHand* rightHand() = 0;

    /**
      \brief Set the pointer to the left hand
    */
    virtual void leftHand(CjrlHand* inLeftHand) = 0;

    /**
    \brief Get a pointer to the left hand
     */
    virtual CjrlHand* leftHand() = 0;

    /**
    \brief Get the hand clench value. 
    This is a scalar value ranging between 0 and 1 which 
    describes the hand clench (0 for open and 1 for closed hand)
     */
    virtual double getHandClench(CjrlHand* inHand) = 0;

    /**
      \brief Set the hand clench value. This is a scalar value 
      ranging between 0 and 1 which describes the hand clench 
      (0 for open and 1 for closed hand)
      \return false if parameter 2 is out of range
    */
    virtual bool setHandClench(CjrlHand* inHand, double inClenchingValue) = 0;

    /**
       \brief Set the pointer to the left foot joint.
    */
    virtual void leftFoot(CjrlFoot* inLeftFoot) = 0;

    /**
        \brief Get a pointer to the left foot.
    */
    virtual CjrlFoot* leftFoot() = 0;

    /**
       \brief Set the pointer to the right foot joint.
    */
    virtual void rightFoot(CjrlFoot* inRightFoot) = 0;

    /**
        \brief Get a pointer to the right foot.
    */
    virtual CjrlFoot* rightFoot() = 0;

    /**
        \brief Set gaze joint
        
        \note  For most humanoid robots, the gaze joint is the head.
    */
    virtual void gazeJoint(CjrlJoint* inGazeJoint) = 0;

    /**
       \brief Get gaze joint
    */
    virtual CjrlJoint* gazeJoint() = 0;

    /**
        \brief Set the gaze orientation and position in the local frame of the gaze joint.
        \return inOrigin a point on the gaze straight line,
        \return inDirection the direction of the gaze joint.
    */
    virtual void gaze(const vector3d& inDirection, const vector3d& inOrigin) = 0;

    /**
        \brief Get a point on the gaze straight line
     */
    virtual const vector3d& gazeOrigin() const = 0;

    /**
        \brief Get the direction of gaze
     */
    virtual const vector3d& gazeDirection() const = 0;



    /**
       \brief Get the jacobian of a joint wrt to internal configuration variables assuming a joint is fixed.
       
        Fixed joint is first fixed joint in vector.
        \return true if there is at least one fixed joint, false otherwise.  
     */
    virtual bool jacobianJointWrtFixedJoint(CjrlJoint* inJoint, matrixNxP& outJacobian) = 0;



    /**
       \@}
    */

    /**
     \name Zero momentum point
    */

    /**
       \brief return the coordinates of the Zero Momentum Point.
    */
    virtual const vector3d& zeroMomentumPoint() const = 0;


    /**
       @}
    */

    /** \name Deprecated methods
      @{
    */

    /**
       \brief Return the distance between the sole of a foot and its joint center

       \deprecated This piece of information has been moved in class CjrlFoot
    */
    virtual double footHeight() const = 0__attribute__ ((deprecated));

    
    /*! @} */
    
};

#endif
