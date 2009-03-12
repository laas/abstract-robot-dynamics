# Olivier Stasse, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST
SET(${${PROJECT_NAME}_SOURCE_DIR}/documentation/docfiles
    ${${PROJECT_NAME}_SOURCE_DIR}/documentation/header.html
    ${${PROJECT_NAME}_SOURCE_DIR}/documentation/footer.html
    ${${PROJECT_NAME}_SOURCE_DIR}/documentation/package.css
    ${${PROJECT_NAME}_SOURCE_DIR}/documentation/pictures/footer.jpg
    ${${PROJECT_NAME}_SOURCE_DIR}/documentation/additionalDoc/package.h
    ${${PROJECT_NAME}_SOURCE_DIR}/documentation/package.dox
    )

# Configure package.dox file
CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/documentation/package.dox.cmake
  ${${PROJECT_NAME}_SOURCE_DIR}/documentation/package.dox
)


ADD_CUSTOM_COMMAND(OUTPUT ${${PROJECT_NAME}_SOURCE_DIR}/documentation/html/index.html
                   COMMAND ${DOXYGEN_EXECUTABLE} \"${${PROJECT_NAME}_SOURCE_DIR}/documentation/package.dox\"
                   DEPENDS ${docfiles}
)

ADD_CUSTOM_TARGET(documentation ALL
                  DEPENDS ${${PROJECT_NAME}_SOURCE_DIR}/documentation/html/index.html)

# doxytag
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.doxytag
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/${PROJECT_NAME}/html)

INSTALL(DIRECTORY ${${PROJECT_NAME}_BINARY_DIR}/documentation/html
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/${PROJECT_NAME}/)
INSTALL(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR}/documentation/pictures
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/${PROJECT_NAME}/html/)

