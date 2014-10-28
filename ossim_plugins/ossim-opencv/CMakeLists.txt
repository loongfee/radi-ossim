PROJECT (opencv-ossim-plugin)
   cmake_minimum_required(VERSION 2.6)

#####
# OpenCV
#####

FIND_PATH(CV_INCLUDE_DIR cv.h
      /usr/include/opencv
      /usr/local/include/opencv/cv/include
      $ENV{ProgramFiles}/OpenCV/cv/include
      $ENV{SystemDrive}/OpenCV/cv/include
      )

FIND_PATH(CXCORE_INCLUDE_DIR cvaux.h
      /usr/include/opencv
      /usr/local/include/opencv/cvaux/include
      $ENV{ProgramFiles}/OpenCV/cvaux/include
      $ENV{SystemDrive}/OpenCV/cvaux/include
      )

FIND_PATH(CVAUX_INCLUDE_DIR cxcore.h
      /usr/include/opencv
      /usr/local/include/opencv/cxcore/include
      $ENV{ProgramFiles}/OpenCV/cxcore/include
      $ENV{SystemDrive}/OpenCV/cxcore/include
      )

FIND_PATH(HIGHGUI_INCLUDE_DIR highgui.h
      /usr/include/opencv
      /usr/local/include/opencv/otherlibs/highgui
      $ENV{ProgramFiles}/OpenCV/otherlibs/highgui
      $ENV{SystemDrive}/OpenCV/otherlibs/highgui
      )

FIND_LIBRARY(CV_LIB NAMES cv PATHS
     /usr/lib
     /usr/local/lib
     /usr/lib/postgresql
     /usr/lib64
     /usr/local/lib64
     $ENV{ProgramFiles}/OpenCV/lib
     $ENV{SystemDrive}/OpenCV/lib
     )

FIND_LIBRARY(CVAUX_LIB NAMES cvaux PATHS
     /usr/lib
     /usr/local/lib
     /usr/lib/postgresql
     /usr/lib64
     /usr/local/lib64
     $ENV{ProgramFiles}/OpenCV/lib
     $ENV{SystemDrive}/OpenCV/lib
     )

FIND_LIBRARY(CXCORE_LIB NAMES cxcore PATHS
     /usr/lib
     /usr/local/lib
     /usr/lib/postgresql
     /usr/lib64
     /usr/local/lib64
     $ENV{ProgramFiles}/OpenCV/lib
     $ENV{SystemDrive}/OpenCV/lib
     )

FIND_LIBRARY(HIGHGUI_LIB NAMES highgui PATHS
     /usr/lib
     /usr/local/lib
     /usr/lib/postgresql
     /usr/lib64
     /usr/local/lib64
     $ENV{ProgramFiles}/OpenCV/lib
     $ENV{SystemDrive}/OpenCV/lib
     )

###
# OSSIM
###

FIND_PATH(OSSIM_INCLUDE_DIR ossim/ossimVersion.h
    /usr/include/ossim
    /c/msys/1.0/include
    c:/msys/1.0/include
    )

FIND_PATH(OSG_INCLUDE_DIR OpenThreads/ScopedLock
    /usr/include/ossim
    /c/msys/1.0/include
    c:/msys/1.0/include
    )

FIND_LIBRARY(OSSIM_LIB NAMES ossim PATHS
    /usr/lib
    /c/msys/1.0/lib
    c:/msys/1.0/lib
    )
	
INCLUDE_DIRECTORIES(
    ${HIGHGUI_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIR}
    ${CXCORE_INCLUDE_DIR}
    ${CVAUX_INCLUDE_DIR}
    ${OSSIM_INCLUDE_DIR}
	${OSG_INCLUDE_DIR}
)
IF(UNIX)
	set(CMAKE_CXX_FLAGS "-g '-DCOMPLEX2=std::complex<double>'")
	set(CMAKE_C_FLAGS "-g")
ENDIF(UNIX)

SUBDIRS(plugin)
SUBDIRS(apps)