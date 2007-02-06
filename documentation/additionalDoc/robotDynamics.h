/** \mainpage 
\section sec_intro Introduction

We propose in the following header files abstract interfaces to define
a robot with dynamics.  The goal is to provide a standard within JRL
developments in order to make packages dealing with humanoid robots
compatible with each other.

\section sec_template Template mechanism

Many robotics functionalities use matrices and vectors. In order to avoid choosing one linear algebra package
and the corresponding data-structure, the abstract interface classes are templated by the following classes:
\li \code Mnxp \endcode for general size matrices,
\li \code M4x4 \endcode for 4 by 4 matrices,
\li \code M3x3 \endcode for 3 by 3 matrices,
\li \code Vn \endcode for general size vectors
\li \code V3 \endcode for vectors of dimension 3

\section sec_howto How it works

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
class Cimpl1DynamicRobot : public CjrlDynamicRobot<> {
public:
  Cimpl1DynamicRobot(...);
  ...
};

//
// Implementation 1 of joint
//
class Cimpl1JointRotation : public CjrlJoint<> {
public:
  Cimpl1JointRotation(...);
  ...
};

//
// Implementation 1 of body
//
class Cimpl1Body : public CjrlBody<> {
public:
  Cimpl1Body(...);
  ...
};


//
// Implementation 2 of dynamic robot
//
class Cimpl2DynamicRobot : public CjrlDynamicRobot<> {
public:
  Cimpl2DynamicRobot(...);
  ...
};

//
// Implementation 2 of joint
//
class Cimpl2JointRotation : public CjrlJoint<> {
public:
  Cimpl2JointRotation(...);
  ...
};

//
// Implementation 2 of body
//
class Cimpl2Body : public CjrlBody<> {
public:
  Cimpl2Body(...);
  ...
};
\endcode
To choose implementation 1, the user will create an object of class Cimpl1DynamicRobot:
\code
CjrlDynamicRobot<>* robot = new Cimpl1DynamicRobot();
\endcode
This seems very simple and reasonable. However, to build the kinematic chain of the robot, the user will then create joints, bodies and for each such operation, he will construct an object of implementation 1:
\code
CjrlJoint<>* joint = new Cimpl1JointRotation(inInitialPosition);
CjlrBody* body = new Cimpl1Body();
\endcode
We now see that several instructions are specific to implementation 1. This makes it then more difficult to choose another implementation.


To solve this problem, we have defined a template class CjrlRobotDynamicsObjectConstructor that takes as template parameters the name of the classes defined by the implementation.
The user of the abstract interface only needs to write the following lines in his source code:
\code
typedef CjrlRobotDynamicsObjectConstructor 
  <Cimpl1dynamicRobot, Cimpl1humanoidDynamicRobot, Cimpl1jointFreeflyer, Cimpl1jointRotation, Cimpl1jointTranslation, Cimpl1body, Mnxp, M4x4, M3x3, Vn, V3> CrobotDynamicConstructor;   
\endcode
This line is the only one that is specific to implementation 1 of the interface.

To create any object of implementation 1 of the abstract interface, the user indeed writes:

\code
CjrlDynamicRobot<>* robot = CrobotDynamicConstructor::createDynamicRobot();

CjrlJoint<>* joint = CrobotDynamicConstructor::createJointFreeflyer(inInitialPosition);

CjrlJoint<>* joint = CrobotDynamicConstructor::createJointRotation(inInitialPosition);
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
Some classes need to be derived into several concrete classes. For instance CjrlJoint<> should be derived into FREEFLYER, ROTATION and TRANSLATION joints.

\subsubsection constructor Constructors

Each implementation of the abstract interface classes should include a constructor as specified in class CjrlRobotDynamicsObjectConstructor.

*/
