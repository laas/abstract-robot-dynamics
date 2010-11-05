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

# Add jrl-mal to include directories.
INCLUDE_DIRECTORIES(${JRL_MAL_INCLUDE_DIRS})

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ABSTRACT_ROBOT_DYNAMICS_TEST(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ABSTRACT_ROBOT_DYNAMICS_TEST NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  # Link against LAPACK.
  TARGET_LINK_LIBRARIES(${NAME} ${LAPACK_LIBRARIES})

  # Link against Boost.
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES})
ENDMACRO(ABSTRACT_ROBOT_DYNAMICS_TEST)

# Simple test.
ABSTRACT_ROBOT_DYNAMICS_TEST(simple)
