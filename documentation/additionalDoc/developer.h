/**
   \page abstractRobotDynamics_developing Developing an implementation

This page provides specifications applicable to any implementation of the abstract interface.

\section derived Deriving abstract classes

The developer of an implementation needs to derive the abstract classes of the interface and to implement the pure virtual methods.
Some classes need to be derived into several concrete classes. For instance CjrlJoint should be derived into FREEFLYER, ROTATION and TRANSLATION joints.

\subsection humanoid_inheritance Inheritance of CjrlHumanoidDynamicRobot implementation

The implementation of CjrlHumanoidDynamicRobot should \b virtually \b inherit from the implementation of CjrlDynamicRobot

\section abstractRobotDynamics_types

Some types or classes need to be provided by the implementation. These types are:
\li <b>vector3d</b> to represent a 3D point or vector,
\li <b>matrix3d</b> to represent a 3 by 3 matrix (for inertia matrix),
\li <b>matrix4d</b> to represent an homogeneous matrix,
\li <b>matrixNxP</b> to represent matrices of any size.

\note From the choices of these types will of course depend the compatibility between an implementation and a module using the interfaces.

\section abstractRobotDynamics_exporting Exporting the name of the classes

Let us assume that the package named <tt>impl1RobotDynamics</tt> provides an implementation of the abstract interface with:
  \li Cimpl1DynamicRobot implements CjrlDynamicRobot,
  \li Cimpl1HumanoidDynamicRobot implements CjrlHumanoidDynamicRobot,
  \li Cimpl1JointFreeFlyerJoint implements a free-flyer joint deriving from CjrlJoint,
  \li Cimpl1JointRotation implements a rotation joint deriving from CjrlJoint,
  \li Cimpl1JointTranslation implements a translation joint deriving from CjrlJoint,
  \li Cimpl1Body implements CjrlBody,

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
#include "impl1RobotDynamics/impl1Body.h"

typedef Cimpl1DynamicRobot CimplDynamicRobot;
typedef Cimpl1HumanoidDynamicRobot CimplHumanoidDynamicRobot;
typedef Cimpl1JointFreeFlyerJoint CimplJointFreeFlyer;
typedef Cimpl1JointRotation CimplJointRotation;
typedef Cimpl1JointTranslation CimplJointTranslation;
typedef Cimpl1Body CimplBody;

#endif
\endcode
Thus, by including this file, the user of the implementation has access to the definitions of the implementation classes and can use them without knowing the name of these classes.

\section constructor Constructors

Each implementation of the abstract interface classes should include a constructor as specified in class CjrlRobotDynamicsObjectConstructor.

*/
