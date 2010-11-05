# Olivier Stasse, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST

cmake_minimum_required(VERSION 2.6)

enable_language(CXX)


# Debug mode for cmake.
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Set version
set(CPACK_PACKAGE_VERSION_MAJOR 1)
set(CPACK_PACKAGE_VERSION_MINOR 16)
set(CPACK_PACKAGE_VERSION_PATCH 0)
SET(PROJECT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}.99)


# name of the project
SET(PROJECT_NAME abstract-robot-dynamics)
SET(PROJECT_CONTACT openrobots@laas)

project(${PROJECT_NAME})

# Check for required packages
# -------------------------
INCLUDE(FindPkgConfig)
pkg_check_modules(MALJRL REQUIRED jrl-mal>=1.7.4 )

SET(PROJECT_REQUIREMENTS "jrl-mal >= 1.7.4")

# To deal with the headers.
#--------------------------
add_subdirectory(interfaces)

# Prepare the documentation
# -------------------------
OPTION(GENERATE_DOC "Generate html documentation" 1)
IF (GENERATE_DOC)
  INCLUDE(FindDoxygen)
  IF(DOXYGEN_FOUND)
    MESSAGE(STATUS "Doxygen found")
	ADD_SUBDIRECTORY(documentation)
  ENDIF(DOXYGEN_FOUND)
ENDIF (GENERATE_DOC)

# Uninstall target.
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

# Prepare description file for pkg-config.
#-----------------------------------------

SET(install_pkg_prefix "\${prefix}")
SET(install_pkg_exec_prefix "\${exec_prefix}")
SET(install_pkg_libdir "\${libdir}")
SET(install_pkg_datarootdir "\${datarootdir}")
SET(install_pkg_include_dir "\${includedir}")

# Install the file in the appropriate place.
SET(install_libdir_pkgconfig lib/pkgconfig)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake
		 ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc )

INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION ${install_libdir_pkgconfig}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)

#---------------
# Package target
#---------------
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "abstract-robot-dynamics - An abstract layer to handle a robot model")
set(CPACK_PACKAGE_VENDOR "LAAS CNRS")
set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
set(CPACK_GENERATOR TGZ)
set(CPACK_GENERATOR DEB)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "jrl-mal (>=1.7.4)")
set(
CPACK_SOURCE_PACKAGE_FILE_NAME
"abstract-robot-dynamics-${PROJECT_VERSION}"
CACHE INTERNAL "tarball basename"
)
set(CPACK_SOURCE_GENERATOR TGZ)
set(CPACK_PACKAGE_CONTACT "Florent Lamiraux")

# The following components are regex's to match anywhere (unless anchored)
# in absolute path + filename to find files or directories to be excluded
# from source tarball.
set(CPACK_SOURCE_IGNORE_FILES
"~$"
"^${PROJECT_SOURCE_DIR}/build/"
"^${PROJECT_SOURCE_DIR}/.git/"
)
#message("CPACK_SOURCE_IGNORE_FILES = ${CPACK_SOURCE_IGNORE_FILES}")
include(CPack)
