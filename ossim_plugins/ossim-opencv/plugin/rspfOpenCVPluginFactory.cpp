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

#include "rspfOpenCVThresholdFilter.h"
#include "rspfOpenCVSobelFilter.h"
#include "rspfOpenCVCannyFilter.h"
#include "rspfOpenCVErodeFilter.h"
#include "rspfOpenCVDilateFilter.h"
#include "rspfOpenCVSmoothFilter.h"
#include "rspfOpenCVLaplaceFilter.h"
#include "rspfOpenCVGoodFeaturesToTrack.h"
#include "rspfOpenCVSURFFeatures.h"
#include "rspfOpenCVStarFeatures.h"
//#include "rspfOpenCVMSERFeatures.h"
#include "rspfOpenCVPyrSegmentation.h"
#include "rspfOpenCVPyrSegmentation.h"
#include "rspfOpenCVPluginFactory.h"


#include <rspf/base/rspfKeywordNames.h>
#include <rspf/base/rspfObjectFactoryRegistry.h>
#include <rspf/base/rspfTrace.h>

RTTI_DEF1(rspfOpenCVPluginFactory,"rspfOpenCVPluginFactory", rspfImageSourceFactoryBase);

static rspfTrace traceDebug("rspfOpenCVPluginFactory:debug");

rspfOpenCVPluginFactory* rspfOpenCVPluginFactory::theInstance=NULL;
 
rspfOpenCVPluginFactory::~rspfOpenCVPluginFactory()
{
   theInstance = NULL;
   rspfObjectFactoryRegistry::instance()->unregisterFactory(this);
}

rspfOpenCVPluginFactory* rspfOpenCVPluginFactory::instance()
{
   if(!theInstance)
   {
      theInstance = new rspfOpenCVPluginFactory;
   }

   return theInstance;
}
 
rspfObject* rspfOpenCVPluginFactory::createObject(const rspfString& name)const
{

   // lets do the filters first
   if( name == STATIC_TYPE_NAME(rspfOpenCVThresholdFilter))
   {
      return new rspfOpenCVThresholdFilter;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVSobelFilter))
   {
      return new rspfOpenCVSobelFilter;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVCannyFilter))
   {
      return new rspfOpenCVCannyFilter;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVDilateFilter))
   {
      return new rspfOpenCVDilateFilter;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVErodeFilter))
   {
      return new rspfOpenCVErodeFilter;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVSmoothFilter))
   {
      return new rspfOpenCVSmoothFilter;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVLaplaceFilter))
   {
      return new rspfOpenCVLaplaceFilter;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVPyrSegmentation))
   {
      return new rspfOpenCVPyrSegmentation;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVGoodFeaturesToTrack))
   {
      return new rspfOpenCVGoodFeaturesToTrack;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVSURFFeatures))
   {
	   return new rspfOpenCVSURFFeatures;
   }
   else if(name == STATIC_TYPE_NAME(rspfOpenCVStarFeatures))
   {
	   return new rspfOpenCVStarFeatures;
   }
   //else if(name == STATIC_TYPE_NAME(rspfOpenCVMSERFeatures))
   //{
	  // return new rspfOpenCVMSERFeatures;
   //}
   //TODO: Add option to create LogPolarFilter
   return NULL;
}

rspfObject* rspfOpenCVPluginFactory::createObject(const rspfKeywordlist& kwl,
                                                   const char* prefix)const
{
   static const char* MODULE = "rspfOpenCVPluginFactory::createSource";
   
   rspfString copyPrefix = prefix;
   rspfObject* result = NULL;
   
   if(traceDebug())
   {
      CLOG << "looking up type keyword for prefix = " << copyPrefix << endl;
   }

   const char* lookup = kwl.find(copyPrefix, "type");
   if(lookup)
   {
       rspfString name = lookup;
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

void rspfOpenCVPluginFactory::getTypeNameList(std::vector<rspfString>& typeList)const
{
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVThresholdFilter));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVSobelFilter));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVCannyFilter));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVDilateFilter));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVErodeFilter));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVSmoothFilter));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVLaplaceFilter));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVGoodFeaturesToTrack));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVSURFFeatures));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVStarFeatures));
  // typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVMSERFeatures));
   typeList.push_back(STATIC_TYPE_NAME(rspfOpenCVPyrSegmentation));

}
 
rspfOpenCVPluginFactory::rspfOpenCVPluginFactory()
   :rspfImageSourceFactoryBase()
{}
 
rspfOpenCVPluginFactory::rspfOpenCVPluginFactory(const rspfOpenCVPluginFactory&)
   :rspfImageSourceFactoryBase()
{}
 
const rspfOpenCVPluginFactory& rspfOpenCVPluginFactory::operator=(rspfOpenCVPluginFactory&)
{
   return *this;
}
