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
    \brief Get the hand clench value. This is a scalar value ranging between 0 and 1 which describes the hand clench (0 for open and 1 for closed hand)
     */
    virtual double getHandClench(CjrlHand* inHand) = 0;

    /**
      \brief Set the hand clench value. This is a scalar value ranging between 0 and 1 which describes the hand clench (0 for open and 1 for closed hand)
    \return false if parameter 2 is out of range
    */
    virtual bool setHandClench(CjrlHand* inHand, double inClenchingValue) = 0;

    /**
       \brief Set the pointer to the left foot joint.
    */
    virtual void leftFoot(CjrlJoint* inLeftFoot) = 0;

    /**
        \brief Get a pointer to the left foot.
    */
    virtual CjrlJoint* leftFoot() = 0;

    /**
       \brief Set the pointer to the right foot joint.
    */
    virtual void rightFoot(CjrlJoint* inRightFoot) = 0;

    /**
        \brief Get a pointer to the right foot.
    */
    virtual CjrlJoint* rightFoot() = 0;

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
    \brief Return the distance between the sole of a foot and its joint center
     */
    virtual double footHeight() const = 0;



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

    // Returns the width of the foot given in parameter:
    // @param WhichFoot : -1 Right foot 1 Left foot.
    // @paran Depth: depth of the foot (X)
    // @param Width: width of the foot (Y), 
    // @param Height: height of the foot (Z),
    // @param 
    // @return -1 if an error occured,
    //  0 otherwise.
    int GetFootSize(int WhichFoot, double &Depth, double &Width,double &Height);
    
    // Returns the length of the tibia
    // @param WhichSide: -1 Right 1 Left.
    double GetTibiaLength(int WhichSide);

    // Returns the length of the femur
    // @param WhichSide: -1 Right 1 Left.
    double GetFemurLength(int WhichSide);

    // Returns the length of the Upper arm
    // @param WhichSide: -1 Right 1 Left.
    double GetUpperArmLength(int WhichSide);

    // Returns the length of the Fore arm
    // @param WhichSide: -1 Right 1 Left.    
    double GetForeArmLength(int WhichSide);

    // Returns the ankle position in the foot coordinate frame
    // @param WhichSide: -1 Right 1 Left.    
    // @return AnklePosition: (X,Y,Z)
    void GetAnklePosition(int WhichSide, double AnklePosition[3]);

    // Returns the position of the Hip regarding the waist's origin.
    // @param WhichSide: -1 Right 1 Left.
    // @ return WaistToHip translation.
    void GetWaistToHip(int WhichSide, double WaistToHip[3]);
    
    // Returns the Hip's length, for instance in HRP-2 the Y-axis
    // for the hip is translated regarding the X and Z axis.
    // @param WhichSide: -1 Right 1 Left.
    // @ return Hip lenght.
    void GetHipLength(int WhichSide,double HipLength[3]);

    /*! \name Joints related methods 
      @{
     */
    
    // Returns the number of joints for the arms */
    int GetArmJointNb(int WhichSide);

    // Returns the joints for one arm */
    const std::vector<int> & GetArmJoints(int WhichSide);

    // Returns the number of joints one leg */
    int GetLegJointNb(int WhichSide);

    // Returns the joints for one leg */
    const std::vector<int> & GetLegJoints(int WhichSide);

    // Returns the number of joints for one foot */
    int GetFootJointNb(int WhichSide);

    // Returns the joints for one foot */
    const std::vector<int> & GetFootJoints(int WhichSide);
    
    
    // Returns the number of joints for the head */
    int GetHeadJointNb();

    // Returns the joints for the head*/
    const std::vector<int> & GetHeadJoints();
    
   // Returns the number of joints for the Chest */
    int GetChestJointNb();

    // Returns the joints for the Chest*/
    const std::vector<int> & GetChestJoints();
 
    // Returns the number of joints for the Upper Body.
    int GetUpperBodyJointNb();

    // Returns the vector of joints index for the
    // Upper body.
    const std::vector<int> & GetUpperBodyJoints();

    // Returns the number of joints for the Waist.
    int GetWaistJointNb();

    /*! \brief Returns the vector of joints index for the
      waist. */
    const std::vector<int> & GetWaistJoints();

    /*! @} */
    
};

#endif
