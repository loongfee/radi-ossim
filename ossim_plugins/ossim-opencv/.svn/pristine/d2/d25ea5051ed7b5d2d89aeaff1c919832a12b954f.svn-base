// Copyright (C) 2010 Argongra 
//
// OSSIM is free software; you can redistribute it and/or
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

#include "ossimOpenCVThresholdFilter.h"
#include "ossimOpenCVSobelFilter.h"
#include "ossimOpenCVCannyFilter.h"
#include "ossimOpenCVErodeFilter.h"
#include "ossimOpenCVDilateFilter.h"
#include "ossimOpenCVSmoothFilter.h"
#include "ossimOpenCVLaplaceFilter.h"
#include "ossimOpenCVGoodFeaturesToTrack.h"
#include "ossimOpenCVSURFFeatures.h"
#include "ossimOpenCVStarFeatures.h"
#include "ossimOpenCVMSERFeatures.h"
#include "ossimOpenCVPyrSegmentation.h"
#include "ossimOpenCVPyrSegmentation.h"
#include "ossimOpenCVPluginFactory.h"


#include <ossim/base/ossimKeywordNames.h>
#include <ossim/base/ossimObjectFactoryRegistry.h>
#include <ossim/base/ossimTrace.h>

RTTI_DEF1(ossimOpenCVPluginFactory,"ossimOpenCVPluginFactory", ossimImageSourceFactoryBase);

static ossimTrace traceDebug("ossimOpenCVPluginFactory:debug");

ossimOpenCVPluginFactory* ossimOpenCVPluginFactory::theInstance=NULL;
 
ossimOpenCVPluginFactory::~ossimOpenCVPluginFactory()
{
   theInstance = NULL;
   ossimObjectFactoryRegistry::instance()->unregisterFactory(this);
}

ossimOpenCVPluginFactory* ossimOpenCVPluginFactory::instance()
{
   if(!theInstance)
   {
      theInstance = new ossimOpenCVPluginFactory;
   }

   return theInstance;
}
 
ossimObject* ossimOpenCVPluginFactory::createObject(const ossimString& name)const
{

   // lets do the filters first
   if( name == STATIC_TYPE_NAME(ossimOpenCVThresholdFilter))
   {
      return new ossimOpenCVThresholdFilter;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVSobelFilter))
   {
      return new ossimOpenCVSobelFilter;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVCannyFilter))
   {
      return new ossimOpenCVCannyFilter;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVDilateFilter))
   {
      return new ossimOpenCVDilateFilter;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVErodeFilter))
   {
      return new ossimOpenCVErodeFilter;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVSmoothFilter))
   {
      return new ossimOpenCVSmoothFilter;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVLaplaceFilter))
   {
      return new ossimOpenCVLaplaceFilter;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVPyrSegmentation))
   {
      return new ossimOpenCVPyrSegmentation;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVGoodFeaturesToTrack))
   {
      return new ossimOpenCVGoodFeaturesToTrack;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVSURFFeatures))
   {
	   return new ossimOpenCVSURFFeatures;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVStarFeatures))
   {
	   return new ossimOpenCVStarFeatures;
   }
   else if(name == STATIC_TYPE_NAME(ossimOpenCVMSERFeatures))
   {
	   return new ossimOpenCVMSERFeatures;
   }
   //TODO: Add option to create LogPolarFilter
   return NULL;
}

ossimObject* ossimOpenCVPluginFactory::createObject(const ossimKeywordlist& kwl,
                                                   const char* prefix)const
{
   static const char* MODULE = "ossimOpenCVPluginFactory::createSource";
   
   ossimString copyPrefix = prefix;
   ossimObject* result = NULL;
   
   if(traceDebug())
   {
      CLOG << "looking up type keyword for prefix = " << copyPrefix << endl;
   }

   const char* lookup = kwl.find(copyPrefix, "type");
   if(lookup)
   {
       ossimString name = lookup;
       result           = createObject(name);
       
       if(result)
       {
         if(traceDebug())
         {
             CLOG << "found source " << result->getClassName() << " now loading state" << endl;
          }
          result->loadState(kwl, copyPrefix.c_str());
       }
       else
       {
         if(traceDebug())
         {
            CLOG << "type not found " << lookup << endl;
         }
      }
   }
   else
   {
       if(traceDebug())
       {
          CLOG << "type keyword not found" << endl;
       }
    }
    return result;
}

void ossimOpenCVPluginFactory::getTypeNameList(std::vector<ossimString>& typeList)const
{
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVThresholdFilter));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVSobelFilter));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVCannyFilter));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVDilateFilter));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVErodeFilter));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVSmoothFilter));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVLaplaceFilter));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVGoodFeaturesToTrack));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVSURFFeatures));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVStarFeatures));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVMSERFeatures));
   typeList.push_back(STATIC_TYPE_NAME(ossimOpenCVPyrSegmentation));

}
 
ossimOpenCVPluginFactory::ossimOpenCVPluginFactory()
   :ossimImageSourceFactoryBase()
{}
 
ossimOpenCVPluginFactory::ossimOpenCVPluginFactory(const ossimOpenCVPluginFactory&)
   :ossimImageSourceFactoryBase()
{}
 
const ossimOpenCVPluginFactory& ossimOpenCVPluginFactory::operator=(ossimOpenCVPluginFactory&)
{
   return *this;
}
