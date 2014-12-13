set(LIB_NAME ossimpdal_plugin)
message( "************** LIBRARY SETUP FOR ossimpdal_plugin******************")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(requiredLibs)

#---
# Need boost:
#---
find_package(Boost)
if(Boost_FOUND)
   include_directories(${Boost_INCLUDE_DIRS})
   set(requiredLibs ${requiredLibs} ${Boost_LIBRARIES})
else(Boost_FOUND)
   message(FATAL_ERROR "Could not find boost")
endif(Boost_FOUND)

#---
# FindPDAL.cmake sets:
# PDAL_INCLUDE_DIR, PDAL_LIBRARY, and PDAL_FOUND
#---
find_package(PDAL)
if(PDAL_FOUND)
   include_directories( ${PDAL_INCLUDE_DIR} )
   set(requiredLibs ${requiredLibs} ${PDAL_LIBRARY} )
else(PDAL_FOUND)
   message(FATAL_ERROR "Could not find pdal")
endif(PDAL_FOUND)

file(GLOB_RECURSE OSSIMPLUGIN_SRCS *.cpp)
file(GLOB_RECURSE OSSIMPLUGIN_HEADERS *.h)

OSSIM_LINK_LIBRARY(${LIB_NAME}
                   COMPONENT_NAME ossim TYPE "${OSSIM_PLUGIN_LINK_TYPE}"
		   LIBRARIES ${OSSIM_LIBRARY} ${requiredLibs}
                   HEADERS "${OSSIMPLUGIN_HEADERS}"
		   SOURCE_FILES "${OSSIMPLUGIN_SRCS}"
                   INSTALL_LIB)
