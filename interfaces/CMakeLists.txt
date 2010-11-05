# Olivier Stasse, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST

SET(${PROJECT_NAME}_HEADERS
	deprecated.hh
	jrlbody.hh
	jrldynamicrobot.hh
	jrlhumanoiddynamicrobot.hh
	jrljoint.hh 
	jrlfoot.hh
	jrlhand.hh
	jrlrigidacceleration.hh 
	jrlrigidvelocity.hh 
	jrlrobotdynamicsobjectconstructor.hh 
)



#----------------------------------------------------
# Install procedure for the header files
#----------------------------------------------------
INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
	DESTINATION include/abstract-robot-dynamics
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
