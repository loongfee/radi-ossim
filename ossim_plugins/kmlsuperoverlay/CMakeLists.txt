set(LIB_NAME ossimkmlsuperoverlay_plugin)
MESSAGE( "************** LIBRARY SETUP FOR ossimkmlsuperoverlay_plugin******************")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(requiredLibs)

find_package(MiniZip)
if(MINIZIP_FOUND)
   include_directories(${MINIZIP_INCLUDE_DIR})
   set(requiredLibs ${requiredLibs} ${MINIZIP_LIBRARIES})
else(MINIZIP_FOUND)
   message(FATAL_ERROR "Could not find minizip")
endif(MINIZIP_FOUND)

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
