/** \mainpage 
\section sec_intro Introduction

We propose in the following header files abstract interfaces to define
a robot with dynamics.  The goal is to provide a standard within JRL
developments in order to make packages dealing with humanoid robots
compatible with each other.

\section sec_howto Principle

\image html interface.png "The goal of the abstract interface is to provide several implementations of the same functions through standardized classes and methods. Each implementation provides derived classes of each abstract class of the interface and implements the pure virtual methods. Users using the interfaces can choose any implementations by instantiating the template allocator." 

\subsection subsec_user For the users

Packages using robot dynamics are guaranted to be compatible to any
package implementing the abstract interfaces, as long as they use the
classes defined here.

\subsubsection construction Object constructor template class

In object oriented programming, one cannot construct an object of an abstract class. Therefore, to create a concrete instance of an abstract class, one needs to choose which derived class (i.e. which implementation) one wants to use.

Let us consider the following example: a user wants to create a dynamic robot defined by the abstract interface. The user thus needs to choose which implementation of the abstract interface he is going to use.
Let us assume that two implementations are available:
\code
//
// Implementation 1 of dynamic robot
//
class Cimpl1DynamicRobot : public CjrlDynamicRobot {
public:
  Cimpl1DynamicRobot(...);
  ...
};

//
// Implementation 1 of joint
//
class Cimpl1JointRotation : public CjrlJoint {
public:
  Cimpl1JointRotation(...);
  ...
};

//
// Implementation 1 of body
//
class Cimpl1Body : public CjrlBody {
public:
  Cimpl1Body(...);
  ...
};


//
// Implementation 2 of dynamic robot
//
class Cimpl2DynamicRobot : public CjrlDynamicRobot {
public:
  Cimpl2DynamicRobot(...);
  ...
};

//
// Implementation 2 of joint
//
class Cimpl2JointRotation : public CjrlJoint {
public:
  Cimpl2JointRotation(...);
  ...
};

//
// Implementation 2 of body
//
class Cimpl2Body : public CjrlBody {
public:
  Cimpl2Body(...);
  ...
};
\endcode
To choose implementation 1, the user will create an object of class Cimpl1DynamicRobot:
\code
CjrlDynamicRobot* robot = new Cimpl1DynamicRobot();
\endcode
This seems very simple and reasonable. However, to build the kinematic chain of the robot, the user will then create joints, bodies and for each such operation, he will construct an object of implementation 1:
\code
CjrlJoint* joint = new Cimpl1JointRotation(inInitialPosition);
CjlrBody* body = new Cimpl1Body();
\endcode
We now see that several instructions are specific to implementation 1. This makes it then more difficult to choose another implementation.


To solve this problem, we have defined a template class CjrlRobotDynamicsObjectConstructor that takes as template parameters the name of the classes defined by the implementation.
The user of the abstract interface only needs to write the following lines in his source code:
\code
typedef CjrlRobotDynamicsObjectConstructor 
  <Cimpl1dynamicRobot, Cimpl1humanoidDynamicRobot, Cimpl1jointFreeflyer, Cimpl1jointRotation, Cimpl1jointTranslation, Cimpl1body> CrobotDynamicConstructor;   
\endcode
This line is the only one that is specific to implementation 1 of the interface.

To create any object of implementation 1 of the abstract interface, the user indeed writes:

\code
CjrlDynamicRobot* robot = CrobotDynamicConstructor::createDynamicRobot();

CjrlJoint* joint = CrobotDynamicConstructor::createJointFreeflyer(inInitialPosition);

CjrlJoint* joint = CrobotDynamicConstructor::createJointRotation(inInitialPosition);
\endcode
where no reference of implementation 1 appears.

\subsubsection method Pure virtual methods

By constructing objects using the template class defined in the above section and by calling only virtual methods defined in the interface classes, the user can
\li build a dynamic model of a robot (humanoid or not)
\li make computation with this dynamic model.
independently of the implementation.

\subsection subsec_developers For the developers

A developer implementing classes deriving from the abstract interface classes can
share his work with the user of the above section.

\subsubsection derived Deriving abstract classes

The developer of an implementation needs to derive the abstract classes of the interface and to implement the pure virtual methods.
Some classes need to be derived into several concrete classes. For instance CjrlJoint should be derived into FREEFLYER, ROTATION and TRANSLATION joints.

\subsubsection abstractRobotDynamics_exporting Exporting the name of the classes

Let us assume that the package named <tt>impl1RobotDynamics</tt> provides an implementation of the abstract interface with:
  \li Cimpl1DynamicRobot implements CjrlDynamicRobot,
  \li Cimpl1HumanoidDynamicRobot implements CjrlHumanoidDynamicRobot,
  \li Cimpl1FreeFlyerJoint implements a free-flyer joint deriving from CjrlJoint,
  \li Cimpl1Rotation implements a rotation joint deriving from CjrlJoint,
  \li Cimpl1Translation implements a translation joint deriving from CjrlJoint,

To make these classes accessible to a user, the names of the classes should be exported in file <tt>impl1RobotDynamics/robotDynamicsImpl.h</tt>. This file should include the following line:
\code
#ifndef ROBOT_DYNAMICS_IMPL_H
#define ROBOT_DYNAMICS_IMPL_H

/*
  Include files defining implementation classes of the abstract interfaces
 */
#include "impl1RobotDynamics/impl1DynamicRobot.h"
#include "impl1RobotDynamics/impl1HumanoidDynamicRobot.h"
#include "impl1RobotDynamics/impl1FreeFlyerJoint.h"
#include "impl1RobotDynamics/impl1Rotation.h"
#include "impl1RobotDynamics/impl1Translation.h"

typedef Cimpl1DynamicRobot CimplDynamicRobot;
typedef Cimpl1HumanoidDynamicRobot CimplHumanoidDynamicRobot;
typedef Cimpl1FreeFlyerJoint CimplJointFreeFlyer;
typedef Cimpl1Rotation CimplJointRotation;
typedef Cimpl1Translation CimplJointTranslation;

#endif
\endcode
Thus, by including this file, the user of the implementation has access to the definitions of the implementation classes and can use them without knowing the name of these classes.

\subsubsection constructor Constructors

Each implementation of the abstract interface classes should include a constructor as specified in class CjrlRobotDynamicsObjectConstructor.

*/
