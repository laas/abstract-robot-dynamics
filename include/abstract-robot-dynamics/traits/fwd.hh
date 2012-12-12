// Copyright 2012 Antonio El Khoury, CNRS
//
// This file is part of abstract-robot-dynamics.
// abstract-robot-dynamics is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// abstract-robot-dynamics is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Lesser Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with abstract-robot-dynamics.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef ABSTRACT_ROBOT_DYNAMICS_TRAITS_FWD_HH
# define ABSTRACT_ROBOT_DYNAMICS_TRAITS_FWD_HH

/// \def ARD_DEFINE_TYPES(T, NAME)
///
/// Create pointer typedefs from class.
///
/// This macro defines new pointer types from \a T and
/// prefixes them with \a NAME.
///
/// \param T class name
/// \param Name desired name
# define ARD_DEFINE_TYPES(T, NAME)				\
  typedef to_pointer<T>::type NAME##Ptr;			\
  typedef to_pointer<const T>::type NAME##ConstPtr

class CjrlJoint;
class CjrlBody;
class CjrlHand;
class CjrlFoot;
class CjrlDynamicRobot;
class CjrlHumanoidDynamicRobot;

#endif //! ABSTRACT_ROBOT_DYNAMICS_TRAITS_FWD_HH
