/*
 *   Copyright (c) 2009 CNRS-AIST
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Olivier STASSE
 *
 */

#ifndef JRL_DYNAMIC_ROBOT_NA_H_
#define JRL_DYNAMIC_ROBOT_NA_H_

#include <robotDynamics/jrlJoint.h>
#include <robotDynamics/jrlDynamicRobot.h>

/**
   \brief Template to implement a non abstract class for a robot with dynamic properties
   from an object factory.
 
   
*/
template <class T>
class CjrlDynamicRobotNA : public virtual CjrlDynamicRobot
{
private:
  CjrlDynamicRobot *m_DR;
public:
  /**
     \name Initialization
     @{
  */
  CjrlDynamicRobotNA(T *inObjectFactory)
  {
    m_DR = inObjectFactory->createDynamicRobot();
  }

  CjrlDynamicRobotNA(CjrlDynamicRobotNA<T> *inDRNA)
  {
    m_DR = inDRNA;
  }

  CjrlDynamicRobotNA()
  {
    m_DR = 0;
  }
  
  /**
     \brief Initialize data-structure necessary to dynamic computations
     This function should be called after building the tree of joints.
  */
  virtual bool initialize()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->initialize();
  }

  /**
  \brief Destructor
   */
  virtual ~CjrlDynamicRobotNA() 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      delete m_DR;
  };
  
  /**
     @}
  */
  /**
     \name Kinematic chain
     @{
  */

  /**
     \brief Set the root joint of the robot.
  */
  virtual void rootJoint(CjrlJoint& inJoint)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      m_DR->rootJoint(inJoint);
  }

  /**
     \brief Get the root joint of the robot.
  */
  virtual CjrlJoint* rootJoint() const
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->rootJoint();

#ifndef NDEBUG
    return (CjrlJoint*) 0;
#endif

  }

  /**
     \brief Get a vector containing all the joints.
  */
  virtual std::vector< CjrlJoint* > jointVector()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->jointVector();

#ifndef NDEBUG
    std::vector<CjrlJoint *> res;
    res.clear();
    return res;
#endif

  }
  
  /**
  \brief Get the chain of joints influencing the relative kinematics between 
  \param inStartJoint and \param inEndJoint.
  */
  virtual std::vector<CjrlJoint*> jointsBetween(const CjrlJoint& inStartJoint, 
						const CjrlJoint& inEndJoint) const
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->jointsBetween(inStartJoint,inEndJoint);

#ifndef NDEBUG
    std::vector<CjrlJoint *> res;
    res.clear();
    return res;
#endif
    
  }
  
  /**
     \brief Get the upper bound for ith dof.
  */
  virtual double upperBoundDof(unsigned int inRankInConfiguration)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif

    return m_DR->upperBoundDof(inRankInConfiguration);

#ifndef NDEBUG
    std::cerr << "m_DR not initialized" << std::endl;
#endif
  }
  /**
     \brief Get the lower bound for ith dof.
  */
  virtual double lowerBoundDof(unsigned int inRankInConfiguration)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif

    return m_DR->lowerBoundDof(inRankInConfiguration);

#ifndef NDEBUG
    std::cerr << "m_DR not initialized" << std::endl;
#endif

  }

  /**
     \brief Compute the upper bound for ith dof using other configuration values if possible.
  */
  virtual double upperBoundDof(unsigned int inRankInConfiguration,
			       const vectorN& inConfig) 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->upperBoundDof(inRankInConfiguration,inConfig);
#ifndef NDEBUG
    std::cerr << "m_DR not initialized" << std::endl;
#endif

  }
  /**
     \brief Compute the lower bound for ith dof using other configuration values if possible.
  */
  virtual double lowerBoundDof(unsigned int inRankInConfiguration,
			       const vectorN& inConfig)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->lowerBoundDof(inRankInConfiguration,inConfig);
#ifndef NDEBUG
    std::cerr << "m_DR not initialized" << std::endl;
#endif
  }

  /**
     \brief Get the number of degrees of freedom of the robot.
  */
  virtual unsigned int numberDof() const
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->numberDof();
#ifndef NDEBUG
    return 0;
#endif
  }

  /**
     \brief Set the joint ordering in the configuration vector
     
     \param inJointVector Vector of the robot joints

     Specifies the order of the joints in the configuration vector. 
     The vector should contain all the joints of the current robot.
  */
  virtual void setJointOrderInConfig(std::vector<CjrlJoint*> inJointVector)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->setJointOrderInConfig(inJointVector);
#ifndef NDEBUG
    return;
#endif

  }

  /**
     @}
  */

  /**
     \name Configuration, velocity and acceleration
  */

  /**
     \brief Set the current configuration of the robot.  

     \param inConfig the configuration vector \f${\bf q}\f$.
       
     \return true if success, false if failure (the dimension of the
     input vector does not fit the number of degrees of freedom of the
     robot).
  */
  virtual bool currentConfiguration(const vectorN& inConfig)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->currentConfiguration(inConfig);
#ifndef NDEBUG
    return false;
#endif
  }

  /**
     \brief Get the current configuration of the robot.

     \return the configuration vector \f${\bf q}\f$.
  */
  virtual const vectorN& currentConfiguration() const
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->currentConfiguration();
#ifndef NDEBUG
#endif
  }

  /**
     \brief Set the current velocity of the robot.  

     \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.

     \return true if success, false if failure (the dimension of the
     input vector does not fit the number of degrees of freedom of the
     robot).
  */
  virtual bool currentVelocity(const vectorN& inVelocity)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->currentVelocity(inVelocity);
#ifndef NDEBUG
    return false;
#endif
    
  }

  /**
     \brief Get the current velocity of the robot.

     \return the velocity vector \f${\bf \dot{q}}\f$.
  */
  virtual const vectorN& currentVelocity() const
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->currentVelocity();
#ifndef NDEBUG
#endif

  }
  /**
     \brief Set the current acceleration of the robot.  

     \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.

     \return true if success, false if failure (the dimension of the
     input vector does not fit the number of degrees of freedom of the
     robot).
  */
  virtual bool currentAcceleration(const vectorN& inAcceleration)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->currentAcceleration(inAcceleration);
#ifndef NDEBUG
    return false;
#endif

  }

  /**
     \brief Get the current acceleration of the robot.

     \return the acceleration vector \f${\bf \ddot{q}}\f$.
  */
  virtual const vectorN& currentAcceleration() const 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->currentAcceleration();
#ifndef NDEBUG
#endif

  }

  /**
     \brief Get the current forces of the robot.

     \return the force vector \f${\bf f}\f$.
  */
  virtual const matrixNxP& currentForces() const
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->currentForces();
  }

  /**
     \brief Get the current torques of the robot.

     \return the torque vector \f${\bf \tau }\f$.
  */
  virtual const matrixNxP& currentTorques() const 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->currentTorques();
  }


  /**
     @}
  */

  /**
     \name Forward kinematics and dynamics
  */


  /**
     \brief Compute forward kinematics.

     Update the position, velocity and accelerations of each
     joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.

  */
  virtual bool computeForwardKinematics()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->computeForwardKinematics();
  }
  

  /**
     \brief Compute the dynamics of the center of mass.

     Compute the linear and  angular momentum and their time derivatives, at the center of mass.
  */
  virtual bool computeCenterOfMassDynamics() 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->computeCenterOfMassDynamics();
  }

  /**
     \brief Get the position of the center of mass.
  */
  virtual const vector3d& positionCenterOfMass() const 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->positionCenterOfMass();
  }

  /**
     \brief Get the velocity of the center of mass.
  */
  virtual const vector3d& velocityCenterOfMass()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->velocityCenterOfMass();
  }

  /**
     \brief Get the acceleration of the center of mass.
  */
  virtual const vector3d& accelerationCenterOfMass()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->accelerationCenterOfMass();
  }

  /**
     \brief Get the linear momentum of the robot.
  */
  virtual const vector3d& linearMomentumRobot()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->linearMomentumRobot();
  }

  /**
     \brief Get the time-derivative of the linear momentum.
  */
  virtual const vector3d& derivativeLinearMomentum()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->derivativeLinearMomentum();
  }

  /**
     \brief Get the angular momentum of the robot at the center of mass.
  */
  virtual const vector3d& angularMomentumRobot()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->angularMomentumRobot();
  }

  /**
     \brief Get the time-derivative of the angular momentum at the center of mass.
  */
  virtual const vector3d& derivativeAngularMomentum()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->derivativeAngularMomentum();
  }

  /**
     \brief Get the total mass of the robot
  */
  virtual double mass() const
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->mass();

  }
  /**
     @}
  */

  /**
     \name Jacobian fonctions
  */

  /**
     \brief Compute the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
  */
  virtual void computeJacobianCenterOfMass()
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      m_DR->computeJacobianCenterOfMass();
  }

  /**
     \brief Get the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
  */
  virtual const matrixNxP& jacobianCenterOfMass() const 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->jacobianCenterOfMass();
  }

  /**
     @}
  */

  /**
     \name Control of the implementation
     @{
  */

  /**
     \brief Whether the specified property in implemented.
  */
  virtual bool isSupported(const std::string &inProperty) 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->isSupported(inProperty);
  }

  /**
     \brief Get property corresponding to command name.

     \param inProperty name of the property.
     \retval outValue value of the property if implemented.

     \note The returned string needs to be cast into the right type (double, int,...).
  */
  virtual bool getProperty(const std::string &inProperty, 
			   std::string& outValue) 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->getProperty(inProperty,
			      outValue);
  }

  /**
     \brief Set property corresponding to command name.

     \param inProperty name of the property.
     \param inValue value of the property.

     \note The value string is obtained by writing the 
     corresponding value in a string (operator<<).
  */
  virtual bool setProperty(std::string &inProperty, 
			   const std::string& inValue) 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->setProperty(inProperty,
			      inValue);

  } 

  /**
     @}
  */
  
  
  /**
  Compute and get position and orientation jacobian
  \param inStartJoint the start of the chain of joints influencing the jacobian.
  \param inEndJoint the joint where the control frame is located.
  \param inFrameLocalPoint the position of the control frame in inEndJoint's local frame.
  \param outjacobian computed jacobian matrix.
  \param offset is the rank of the column of 
  \param outjacobian where writing of jacobian begins.
  \param inIncludeStartFreeFlyer is an option to include the 
  contribution of a fictive freeflyer superposed with \param inStartJoint
  
  \return false if matrix has inadequate size. Number of columns 
  in matrix \param outJacobian must be at least numberDof() if inIncludeStartFreeFlyer = true. 
  It must be at least numberDof()-6 otherwise.
  */
  virtual bool getJacobian(const CjrlJoint& inStartJoint, 
			   const CjrlJoint& inEndJoint, 
			   const vector3d& inFrameLocalPosition, 
			   matrixNxP& outjacobian, 
			   unsigned int offset = 0, 
			   bool inIncludeStartFreeFlyer = true) 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->getJacobian(inStartJoint,
				inEndJoint,
				inFrameLocalPosition,
				outjacobian,
				offset,
				inIncludeStartFreeFlyer);
  }
  
  virtual bool getPositionJacobian(const CjrlJoint& inStartJoint, 
				   const CjrlJoint& inEndJoint, 
				   const vector3d& inFrameLocalPosition, 
				   matrixNxP& outjacobian, 
				   unsigned int offset = 0, 
				   bool inIncludeStartFreeFlyer = true)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->getPositionJacobian(inStartJoint,
					inEndJoint,
					inFrameLocalPosition,
					outjacobian,
					offset,
					inIncludeStartFreeFlyer);
  }
  
  virtual bool getOrientationJacobian(const CjrlJoint& inStartJoint, 
				      const CjrlJoint& inEndJoint, 
				      matrixNxP& outjacobian, 
				      unsigned int offset = 0, 
				      bool inIncludeStartFreeFlyer = true) 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->getOrientationJacobian(inStartJoint, 
					   inEndJoint, 
					   outjacobian, 
					   offset, 
					   inIncludeStartFreeFlyer);
  }


  virtual bool getJacobianCenterOfMass(const CjrlJoint& inStartJoint, 
				       matrixNxP& outjacobian, 
				       unsigned int offset = 0, 
				       bool inIncludeStartFreeFlyer = true)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->getJacobianCenterOfMass(inStartJoint,
					    outjacobian,
					    offset,
					    inIncludeStartFreeFlyer);
  }

  /*! \name Inertia matrix related methods 
   @{ */
  /*! \brief Compute the inertia matrix of the robot according wrt \f${\bf q}\f$.
   */
  virtual void computeInertiaMatrix() 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->computeInertiaMatrix();
  }

  /*! \brief Get the inertia matrix of the robot according wrt \f${\bf q}\f$.
   */
  virtual const matrixNxP& inertiaMatrix() const
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->inertiaMatrix();
  }
    /*! @} */

  /*! \name Actuated joints related methods.  
    @{
   */

  /** 
      \brief Returns the list of actuated joints. 
   */
  virtual const std::vector<CjrlJoint*>& getActuatedJoints() const 
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->getActuatedJoints();
  }

  /**
     \brief Specifies the list of actuated joints. 
  */
  virtual void setActuatedJoints(std::vector<CjrlJoint*>& lActuatedJoints)
  {
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->setActuatedJoints(lActuatedJoints);
  }

  /*! 
    @} 
  */


  /*! \brief Compute Speciliazed InverseKinematics between two joints. 
    
    Specialized means that this method can be re implemented to be
    extremly efficient and used the particularity of your robot.
    For instance in some case, it is possible to use an exact inverse
    kinematics to compute a set of articular value.
    
    This method does not intend to replace an architecture computing
    inverse kinematics through the Jacobian.

    jointRootPosition and jointEndPosition have to be expressed in the same
    frame. 

    \param[in] jointRoot: The root of the joint chain for which the specialized
    inverse kinematics should be computed.

    \param[in] jointEnd: The end of the joint chain for which the specialized 
    inverse kinematics should be computed.

    \param[in] jointRootPosition: The desired position of the root. 
    
    \param[in] jointEndPosition: The end position of the root.
    
    \param[out] q: Result i.e. the articular values.
  */
  virtual bool ComputeSpecializedInverseKinematics(const CjrlJoint & jointRoot,
						   const CjrlJoint & jointEnd,
						   const matrix4d & jointRootPosition,
						   const matrix4d & jointEndPosition,
						   vectorN &q)
  { 
#ifndef NDEBUG
    if (m_DR!=0)
#endif
      return m_DR->ComputeSpecializedInverseKinematics(jointRoot,
						       jointEnd,
						       jointRootPosition,
						       jointEndPosition,
						       q);
  }
  
  
};


#endif /* JRL_DYNAMIC_ROBOT_NA_H_ */
