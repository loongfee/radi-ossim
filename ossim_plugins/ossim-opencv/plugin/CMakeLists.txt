###
# Create OpenCV Library Plugin for OSSIM
###

FILE(GLOB_RECURSE OSSIMPLUGIN_SRCS *.cpp)
FILE(GLOB_RECURSE OSSIMPLUGIN_HEADERS *.h)


ADD_LIBRARY(ossimopencv
	${OSSIMPLUGIN_SRCS}
)

ADD_LIBRARY(ossimopencv_plugin SHARED
	${OSSIMPLUGIN_SRCS}
)

TARGET_LINK_LIBRARIES(ossimopencv_plugin 
	${HIGHGUI_LIB}
    ${CV_LIB}
    ${CXCORE_LIB}
    ${CVAUX_LIB}
    ${OSSIM_LIB})

IF(UNIX)
	ADD_CUSTOM_COMMAND(OUTPUT ossimRegPluginLinux
		COMMAND g++ -shared -o libossimopencv_plugin.so ./CMakeFiles/ossimopencv.dir/*.o
			${HIGHGUI_LIB}
				${CV_LIB}
				${CXCORE_LIB}
				${CVAUX_LIB}
				${OSSIM_LIB}
				-I${HIGHGUI_INCLUDE_DIR}
				-I${CV_INCLUDE_DIR}
				-I${CXCORE_INCLUDE_DIR}
				-I${CVAUX_INCLUDE_DIR}
				-I${OSSIM_INCLUDE_DIR}
				DEPENDS ossimopencv
		)

	ADD_CUSTOM_TARGET(ossim DEPENDS ossimRegPluginLinux)
ENDIF(UNIX)

