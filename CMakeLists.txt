# Copyright 2008, 2009, 2010, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of abstract-robot-dynamics.
# abstract-robot-dynamics is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# abstract-robot-dynamics is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# abstract-robot-dynamics.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME abstract-robot-dynamics)
SET(PROJECT_VERSION 1.16.0.99)
SET(PROJECT_DESCRIPTION "JRL mathematical tools")
SET(PROJECT_URL "http://github.com/laas/abstract-robot-dynamics")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(BOOST_COMPONENTS unit_test_framework)
SEARCH_FOR_BOOST()
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.7.4")

SET(${PROJECT_NAME}_HEADERS
  include/abstract-robot-dynamics.hh
  include/abstract-robot-dynamics/body.hh
  include/abstract-robot-dynamics/dynamic-robot.hh
  include/abstract-robot-dynamics/foot.hh
  include/abstract-robot-dynamics/fwd.hh
  include/abstract-robot-dynamics/hand.hh
  include/abstract-robot-dynamics/humanoid-dynamic-robot.hh
  include/abstract-robot-dynamics/io.hh
  include/abstract-robot-dynamics/joint.hh
  include/abstract-robot-dynamics/rigid-acceleration.hh
  include/abstract-robot-dynamics/rigid-velocity.hh
  include/abstract-robot-dynamics/robot-dynamics-object-constructor.hh
  )

SETUP_PROJECT()
SETUP_PROJECT_CPACK()

ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
