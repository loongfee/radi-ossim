//----------------------------------------------------------------------------
//
// File: ossimPdalReaderFactory.cpp
// 
// License:  See top level LICENSE.txt file
//
// Author:  David Burken
//
// Description:
//
// OSSIM Point Data Abstractin Library(PDAL) reader factory.
// 
//----------------------------------------------------------------------------
// $Id$

#include "ossimPdalReaderFactory.h"
#include "ossimPdalReader.h"
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimString.h>
#include <ossim/imaging/ossimImageHandler.h>
#include <ossim/base/ossimTrace.h>
#include <ossim/base/ossimKeywordNames.h>

static const ossimTrace traceDebug("ossimPdalReaderFactory:debug");

RTTI_DEF1(ossimPdalReaderFactory,
          "ossimPdalReaderFactory",
          ossimImageHandlerFactoryBase);

ossimPdalReaderFactory* ossimPdalReaderFactory::theInstance = 0;

ossimPdalReaderFactory::~ossimPdalReaderFactory()
{
   theInstance = 0;
}

ossimPdalReaderFactory* ossimPdalReaderFactory::instance()
{
   if(!theInstance)
   {
      theInstance = new ossimPdalReaderFactory;
   }
   return theInstance;
}
   
ossimImageHandler* ossimPdalReaderFactory::open(
   const ossimFilename& fileName, bool openOverview)const
{
   if(traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "ossimPdalReaderFactory::open(filename) DEBUG: entered..."
         << "\ntrying ossimPdalReader"
         << std::endl;
   }
   
   ossimRefPtr<ossimImageHandler> reader = 0;

   reader = new ossimPdalReader;
   reader->setOpenOverviewFlag(openOverview);
   if(reader->open(fileName) == false)
   {
      reader = 0;
   }
   
   if(traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "ossimPdalReaderFactory::open(filename) DEBUG: leaving..."
         << std::endl;
   }
   
   return reader.release();
}

ossimImageHandler* ossimPdalReaderFactory::open(const ossimKeywordlist& kwl,
                                               const char* prefix)const
{
   if(traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "ossimPdalReaderFactory::open(kwl, prefix) DEBUG: entered..."
         << "Trying ossimPdalReader"
         << std::endl;
   }

   ossimRefPtr<ossimImageHandler> reader = new ossimPdalReader;
   if(reader->loadState(kwl, prefix) == false)
   {
      reader = 0;
   }
   
   if(traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "ossimPdalReaderFactory::open(kwl, prefix) DEBUG: leaving..."
         << std::endl;
   }
   
   return reader.release();
}

ossimObject* ossimPdalReaderFactory::createObject(
   const ossimString& typeName)const
{
   ossimRefPtr<ossimObject> result = 0;
   if(typeName == "ossimPdalReader")
   {
      // result = new ossimPdalReader;
   }
   return result.release();
}

ossimObject* ossimPdalReaderFactory::createObject(const ossimKeywordlist& kwl,
                                                 const char* prefix)const
{
   return this->open(kwl, prefix);
}
 
void ossimPdalReaderFactory::getTypeNameList(std::vector<ossimString>& typeList)const
{
   typeList.push_back(ossimString("ossimPdalReader"));
}

void ossimPdalReaderFactory::getSupportedExtensions(
   ossimImageHandlerFactoryBase::UniqueStringList& extensionList)const
{
   extensionList.push_back(ossimString("las"));
   extensionList.push_back(ossimString("laz"));   
}

ossimPdalReaderFactory::ossimPdalReaderFactory(){}

ossimPdalReaderFactory::ossimPdalReaderFactory(const ossimPdalReaderFactory&){}

void ossimPdalReaderFactory::operator=(const ossimPdalReaderFactory&){}
