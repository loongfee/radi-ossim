set(LIB_NAME ossimpng_plugin)
MESSAGE( "************** LIBRARY SETUP FOR ossimpng_plugin******************")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(requiredLibs)

find_package(PNG)
IF(PNG_FOUND)
   INCLUDE_DIRECTORIES( ${PNG_INCLUDE_DIR} )
   SET(requiredLibs ${requiredLibs} ${PNG_LIBRARIES} )
ELSE(PNG_FOUND)
   MESSAGE(FATAL_ERROR "Could not find png")
ENDIF(PNG_FOUND)

MESSAGE( STATUS "PNG_INCLUDE_DIR = ${PNG_INCLUDE_DIR}")
MESSAGE( STATUS "PNG_LIBRARIES   = ${PNG_LIBRARIES}")

find_package(ZLIB)
IF(ZLIB_FOUND)
   INCLUDE_DIRECTORIES( ${ZLIB_INCLUDE_DIR} )
   SET(requiredLibs ${requiredLibs} ${ZLIB_LIBRARIES} )
ELSE(ZLIB_FOUND)
   MESSAGE(FATAL_ERROR "Could not find zlib")
ENDIF(ZLIB_FOUND)

find_package(TIFF)
IF(TIFF_FOUND)
   INCLUDE_DIRECTORIES( ${TIFF_INCLUDE_DIR} )
   SET(requiredLibs ${requiredLibs} ${TIFF_LIBRARIES} )
ELSE(TIFF_FOUND)
   MESSAGE(FATAL_ERROR "Could not find tiff")
ENDIF(TIFF_FOUND)

FILE(GLOB_RECURSE OSSIMPLUGIN_SRCS *.cpp)
FILE(GLOB_RECURSE OSSIMPLUGIN_HEADERS *.h)

####
# OSSIMPLUGINSMAKINGDLL controls dll linkage on windows.  
# Adding this sets OSSIM_PLUGINS_DLL #define TO "__declspec(dllexport)".
###
add_definitions("-DOSSIMPLUGINSMAKINGDLL")

OSSIM_LINK_LIBRARY(${LIB_NAME}
                   COMPONENT_NAME ossim TYPE "${OSSIM_PLUGIN_LINK_TYPE}"
		   LIBRARIES ${OSSIM_LIBRARY} ${requiredLibs}
                   HEADERS "${OSSIMPLUGIN_HEADERS}"
		   SOURCE_FILES "${OSSIMPLUGIN_SRCS}"
                   INSTALL_LIB)
