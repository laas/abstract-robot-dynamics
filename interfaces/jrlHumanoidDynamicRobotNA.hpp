/*
 *   Copyright (c) 2009 CNRS-AIST
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Olivier STASSE

 */

#ifndef JRL_HUMANOID_DYNAMIC_ROBOT_NA_H_
#define JRL_HUMANOID_DYNAMIC_ROBOT_NA_H_

#include <robotDynamics/jrlHumanoidDynamicRobot.h>

#include "jrlDynamicRobotNA.hpp"
/**
   \brief Template to implement a non abstract class describing a humanoid robot with dynamics.
   This template takes a class implementing the methods of the template
   CjrlRobotDynamicsObjectConstructor.
*/
template <class T>
class CjrlHumanoidDynamicRobotNA : public  virtual CjrlHumanoidDynamicRobot, 
				   public CjrlDynamicRobotNA<T>
{
private:
  CjrlHumanoidDynamicRobot *m_HDR;

public:

  CjrlHumanoidDynamicRobotNA(T * inObjectFactory)
  {
    m_HDR = inObjectFactory->createhumanoidDynamicRobot();
    CjrlDynamicRobotNA<T>(m_HDR);
  }

  CjrlHumanoidDynamicRobotNA(CjrlHumanoidDynamicRobotNA<T> *inHDRNA )
  {
    m_HDR = inHDRNA;
    CjrlDynamicRobotNA<T>(m_HDR);
  }

  CjrlHumanoidDynamicRobotNA()
  {
    m_HDR=0;
    CjrlDynamicRobotNA<T>(m_HDR);
  }
  /**
     \brief Destructor
  */
  virtual ~CjrlHumanoidDynamicRobotNA()
  {
    // m_HDR should be deleted through DynamicRobot.
  }

  /**
     \name Joints specific to humanoid robots
  */
  
  /**
     \brief Set the pointer to the waist.
  */
  virtual void waist(CjrlJoint* inWaist)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      m_HDR->waist(inWaist);

  }

  /**
     \brief Get a pointer to the waist.
  */
  virtual CjrlJoint* waist()
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->waist();

  }

  /**
     \brief Set the pointer to the chest.
     
     \note for some humanoid robots, the waist and the chest are the same joints.
  */
  virtual void chest(CjrlJoint* inChest)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      m_HDR->chest(inChest);
  }

  /**
     \brief Get a pointer to the chest.
     
     \note for some humanoid robots, the waist and the chest are the same joints.
  */
  virtual CjrlJoint* chest()
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->chest();
  }

  /**
     \brief Set the pointer to the left wrist joint.
  */
  virtual void leftWrist(CjrlJoint* inLeftWrist)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      m_HDR->leftWrist(inLeftWrist);
  }

  /**
     \brief Get a pointer to the left wrist.
  */
  virtual CjrlJoint* leftWrist() 
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->leftWrist();
  }

  /**
     \brief Set the pointer to the right wrist joint.
  */
  virtual void rightWrist(CjrlJoint* inRightWrist)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      m_HDR->rightWrist(inRightWrist);
  }
  
  /**
     \brief Get a pointer to the right wrist.
  */
  virtual CjrlJoint* rightWrist()
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->rightWrist();
  }

  /**
     \brief Set the pointer to the right hand
  */
  virtual void rightHand(CjrlHand* inRightHand)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      m_HDR->rightHand(inRightHand);
  }

  /**
     \brief Get a pointer to the right hand
  */
  virtual CjrlHand* rightHand()
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->rightHand();
  }
  
  /**
     \brief Set the pointer to the left hand
  */
  virtual void leftHand(CjrlHand* inLeftHand)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      m_HDR->leftHand(inLeftHand);
  }
  
  /**
     \brief Get a pointer to the left hand
  */
  virtual CjrlHand* leftHand()
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->leftHand();

  }
  
  /**
     \brief Get the hand clench value. 
     This is a scalar value ranging between 0 and 1 which 
     describes the hand clench (0 for open and 1 for closed hand)
  */
  virtual double getHandClench(CjrlHand* inHand)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->getHandClench(inHand);
  }
  
  /**
     \brief Set the hand clench value. This is a scalar value 
     ranging between 0 and 1 which describes the hand clench 
     (0 for open and 1 for closed hand)
     \return false if parameter 2 is out of range
  */
  virtual bool setHandClench(CjrlHand* inHand, double inClenchingValue)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->setHandClench(inHand,inClenchingValue);
  }
  
  /**
     \brief Set the pointer to the left foot joint.
  */
  virtual void leftFoot(CjrlFoot* inLeftFoot)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->leftFoot(inLeftFoot);
  }
  
  /**
     \brief Get a pointer to the left foot.
  */
  virtual CjrlFoot* leftFoot()
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->leftFoot();    
  }

  /**
     \brief Set the pointer to the right foot joint.
  */
  virtual void rightFoot(CjrlFoot* inRightFoot)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->rightFoot(inRightFoot);    
  }

  /**
     \brief Get a pointer to the right foot.
  */
  virtual CjrlFoot* rightFoot()
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->rightFoot();
  }

  /**
     \brief Set gaze joint
        
     \note  For most humanoid robots, the gaze joint is the head.
  */
  virtual void gazeJoint(CjrlJoint* inGazeJoint)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      m_HDR->gazeJoint(inGazeJoint);
  }

  /**
     \brief Get gaze joint
  */
  virtual CjrlJoint* gazeJoint()
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->gazeJoint();
  }

  /**
     \brief Set the gaze orientation and position in the local frame of the gaze joint.
     \return inOrigin a point on the gaze straight line,
     \return inDirection the direction of the gaze joint.
  */
  virtual void gaze(const vector3d& inDirection, const vector3d& inOrigin)
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      m_HDR->gaze(inDirection,inOrigin);

  }

  /**
     \brief Get a point on the gaze straight line
  */
  virtual const vector3d& gazeOrigin() const
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->gazeOrigin();
  }

  /**
     \brief Get the direction of gaze
  */
  virtual const vector3d& gazeDirection() const
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
    return m_HDR->gazeOrigin();
  }

  /**
     \@}
  */

  /**
     \name Zero momentum point
  */

  /**
     \brief return the coordinates of the Zero Momentum Point.
  */
  virtual const vector3d& zeroMomentumPoint() const
  {
#ifndef NDEBUG
    if (m_HDR!=0)
#endif
      return m_HDR->zeroMomentumPoint();
  }


  /**
     \brief Return the distance between the sole of a foot and its joint center
     
     \deprecated This piece of information has been moved in class CjrlFoot
    */
  virtual double footHeight() const  __attribute__ ((deprecated))
  {
    return 0;
  }

    
    /*! @} */

  /**
     @}
  */
    
};

#endif
