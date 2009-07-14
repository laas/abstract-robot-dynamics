# Olivier Stasse, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST

SET(${PROJECT_NAME}_HEADERS
	deprecated.h
	jrlBody.h 
	jrlDynamicRobot.h 
	jrlHumanoidDynamicRobot.h 
	jrlJoint.h 
	jrlFoot.h
	jrlHand.h 
	jrlRigidAcceleration.h 
	jrlRigidVelocity.h 
	jrlRobotDynamicsObjectConstructor.h 
)



#----------------------------------------------------
# Install procedure for the header files
#----------------------------------------------------
INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include/robotDynamics
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
