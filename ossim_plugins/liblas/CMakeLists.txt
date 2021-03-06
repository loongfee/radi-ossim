set(LIB_NAME ossimliblas_plugin)
MESSAGE( "************** LIBRARY SETUP FOR ossimliblas_plugin******************")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(requiredLibs)

find_package(LIBLAS)
IF(LIBLAS_FOUND)
   INCLUDE_DIRECTORIES( ${LIBLAS_INCLUDE_DIR} )
   SET(requiredLibs ${requiredLibs} ${LIBLAS_LIBRARIES} )
ELSE(LIBLAS_FOUND)
   MESSAGE(FATAL_ERROR "Could not find liblas")
ENDIF(LIBLAS_FOUND)

MESSAGE( STATUS "LIBLAS_INCLUDE_DIR  = ${LIBLAS_INCLUDE_DIR}" )
MESSAGE( STATUS "LIBLAS_LIBRARIES    = ${LIBLAS_LIBRARIES}" )

FILE(GLOB_RECURSE OSSIMPLUGIN_SRCS *.cpp)
FILE(GLOB_RECURSE OSSIMPLUGIN_HEADERS *.h)

OSSIM_LINK_LIBRARY(${LIB_NAME}
                   COMPONENT_NAME ossim TYPE "${OSSIM_PLUGIN_LINK_TYPE}"
		   LIBRARIES ${OSSIM_LIBRARY} ${requiredLibs}
                   HEADERS "${OSSIMPLUGIN_HEADERS}"
		   SOURCE_FILES "${OSSIMPLUGIN_SRCS}"
                   INSTALL_LIB)
