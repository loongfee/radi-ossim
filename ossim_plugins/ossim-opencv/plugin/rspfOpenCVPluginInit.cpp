// Copyright (C) 2010 Argongra 
//
// RSPF is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License 
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
//
// You should have received a copy of the GNU General Public License
// along with this software. If not, write to the Free Software 
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-
// 1307, USA.
//
// See the GPL in the COPYING.GPL file for more details.
//
//*************************************************************************

/** 
 \mainpage OpenCV Plugin for RSPF 

 \section intro_sec Introduction
 
 This project was created to provide developers a framekwork to use OpenCV functions in RSPF image processing chains.  

 "OpenCV (Open Source Computer Vision) is a library of programming functions for real time computer vision."

 To learn more about OpenCV, visit
 
 - http://opencv.willowgarage.com/wiki/Welcome
 - http://sourceforge.net/projects/opencvlibrary/ 

 "Open Source Software Image Map (RSPF) is a high performance engine for remote sensing, image processing, geographical information systems and photogrammetry."
 
 Information about OpenCV can be found at:
 - http://www.rspf.org

 \section install_sec Installation
 
 Step 1: Download the latest project source code from svn

 \code 
 >> svn co http://opencv-rspf-plugin.googlecode.com/svn/trunk/ opencv-rspf-plugin-read-only
 \endcode

 Step 2: Build OpenCV plugin 

 \code
 >> cd /path/to/source/ 
 >> cmake -G "Unix Makefiles"
 >> make 
 \endcode
 
 Step 3: Add the plugin path in "rspf_preferences" file 

 To add the OpenCV plugin to the RSPF plugin bridge support, add: 
 \code 
 ...
 plugin.file1: /path/to/source/librspfopencv_plugin.so
 ...
 \endcode

 Don't forget to set the RSPF_PREFS_FILE variable pointing to the "rspf_preferences" file:  
 \code 
 >> export RSPF_PREFS_FILE=path/to/file/rspf_preferences
 \endcode
 
 Step 4: Check the plugin is successfully loaded

 \code 
  >> rspf-info --plugins
 \endcode

 \section doc_sec Documentation

 Under the current project source code folder, type: 
 \code 
 >> doxygen doxyfile
 \endcode
 It will generate an on-line documentation browser (in HTML) from a the documented source files.

 **/

#include "rspfOpenCVPluginFactory.h"
#include "rspfPluginConstants.h"

#include <rspf/imaging/rspfImageSourceFactoryRegistry.h>
#include <rspf/plugin/rspfSharedObjectBridge.h>
#include <rspf/base/rspfString.h>

extern "C"
{
  rspfSharedObjectInfo myInfo;
  rspfString theDescription;
  std::vector<rspfString> theObjList;

  const char* getDescription()
  {
    return theDescription.c_str();
  }

  int getNumberOfClassNames()
  {
      return (int)theObjList.size();
  }

  const char* getClassName(int idx)
  {
      if(idx < (int)theObjList.size())
	{
	  return theObjList[idx].c_str();
	}
      return 0;
  }

  RSPF_PLUGINS_DLL  void rspfSharedLibraryInitialize(rspfSharedObjectInfo** info)
  {
    myInfo.getDescription        = getDescription;
    myInfo.getNumberOfClassNames = getNumberOfClassNames;
    myInfo.getClassName          = getClassName;
    *info = &myInfo;
    
    theDescription = rspfString("OpenCV Plugin");
    
    theObjList.push_back("rspfOpenCVCannyFilter");
    theObjList.push_back("rspfOpenCVDilateFilter");
    theObjList.push_back("rspfOpenCVErodeFilter");
    theObjList.push_back("rspfOpenCVLaplaceFilter");
    theObjList.push_back("rspfOpenCVSmoothFilter");
    theObjList.push_back("rspfOpenCVSobelFilter");
    theObjList.push_back("rspfOpenCVThresholdFilter");
    theObjList.push_back("rspfOpenCVGoodFeaturesToTrack");
    theObjList.push_back("rspfOpenCVPyrSegmentation");

    rspfImageSourceFactoryRegistry::instance()->registerFactory(rspfOpenCVPluginFactory::instance());

  }

  RSPF_PLUGINS_DLL void rspfSharedLibraryFinalize()
  {
    rspfImageSourceFactoryRegistry::instance()->unregisterFactory(rspfOpenCVPluginFactory::instance());
  }

}


