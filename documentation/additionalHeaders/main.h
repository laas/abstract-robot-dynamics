/** \mainpage 
\section sec_intro Introduction

We propose in the following header files abstract interfaces to define
a robot with dynamics.  The goal is to provide a standard within JRL
developments in order to make packages dealing with humanoid robots
compatible with each other.

\section sec_howto How it works

\subsection subsec_user For the users

Packages using robot dynamics are guaranted to be compatible to any
package implementing the abstract interfaces, as long as they use the
classes defined here.

\subsection subsec_developpers For the developers

A developper implementing classes deriving from the abstract interface classes can
share his work with the user of the above section.

For instance, one implementation could be the class PatterGeneratorJRL::dynamicMultiBody:

class PatterGeneratorJRL::dynamicMultiBody: public CjrlDynamicRobot {
  ...
};

*/
