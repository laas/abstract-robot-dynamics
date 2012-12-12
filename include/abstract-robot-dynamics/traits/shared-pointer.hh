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

#ifndef ABSTRACT_ROBOT_DYNAMICS_TRAITS_SHARED_POINTER_HH
# define ABSTRACT_ROBOT_DYNAMICS_TRAITS_SHARED_POINTER_HH

#ifdef ABSTRACT_ROBOT_DYNAMICS_POINTER_TRAIT
# error "A pointer trait header has already been included! Include default-pointer.hh or shared-pointer.hh but not both."
#endif // ABSTRACT_ROBOT_DYNAMICS_POINTER_TRAIT
#define ABSTRACT_ROBOT_DYNAMICS_POINTER_TRAIT

# include <abstract-robot-dynamics/traits/fwd.hh>

template<typename T>
struct to_pointer;

template<typename T>
struct to_pointer<const T>
{
  typedef const boost::shared_ptr<T> type;
};

template<typename T>
struct to_pointer
{
  typedef boost::shared_ptr<T> type;
};

ARD_DEFINE_TYPES(CjrlJoint, JrlJoint);
ARD_DEFINE_TYPES(CjrlBody, JrlBody);
ARD_DEFINE_TYPES(CjrlHand, JrlHand);
ARD_DEFINE_TYPES(CjrlFoot, JrlFoot);
ARD_DEFINE_TYPES(CjrlDynamicRobot, JrlDynamicRobot);
ARD_DEFINE_TYPES(CjrlHumanoidDynamicRobot, JrlHumanoidDynamicRobot);

#endif //! ABSTRACT_ROBOT_DYNAMICS_TRAITS_SHARED_POINTER_HH
