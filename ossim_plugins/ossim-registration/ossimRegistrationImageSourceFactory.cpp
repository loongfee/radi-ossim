#include "ossimRegistrationImageSourceFactory.h"
#include <ossim/base/ossimTrace.h>
#include "ossimSquareFunction.h"

#include <ossim/base/ossimObjectFactoryRegistry.h>

RTTI_DEF1(ossimRegistrationImageSourceFactory, "ossimRegistrationImageSourceFactory", ossimImageSourceFactoryBase);
ossimRegistrationImageSourceFactory* ossimRegistrationImageSourceFactory::theInstance = 0;

static ossimTrace traceDebug("ossimRegistrationImageSourceFactory:debug");

ossimRegistrationImageSourceFactory::ossimRegistrationImageSourceFactory()
{
   theInstance = this;
}

ossimRegistrationImageSourceFactory::ossimRegistrationImageSourceFactory(const ossimRegistrationImageSourceFactory&)
{
}

const ossimRegistrationImageSourceFactory& ossimRegistrationImageSourceFactory::operator=(ossimRegistrationImageSourceFactory&)
{
   return *this;
}

ossimRegistrationImageSourceFactory::~ossimRegistrationImageSourceFactory()
{
   theInstance = 0;
   ossimObjectFactoryRegistry::instance()->unregisterFactory(this);
}

ossimRegistrationImageSourceFactory* ossimRegistrationImageSourceFactory::instance()
{
   if(!theInstance)
   {
      theInstance = new ossimRegistrationImageSourceFactory;
   }

   return theInstance;
}

ossimObject* ossimRegistrationImageSourceFactory::createObject(const ossimString& name)const
{
   if(name == STATIC_TYPE_NAME(ossimSquareFunction))
   {
      return new ossimSquareFunction;
   }

   return 0;
}

ossimObject* ossimRegistrationImageSourceFactory::createObject(const ossimKeywordlist& kwl,
                                                               const char* prefix)const
{
   static const char* MODULE = "ossimImageSourceFactory::createSource";
   
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

void ossimRegistrationImageSourceFactory::getTypeNameList(std::vector<ossimString>& typeList)const
{
   typeList.push_back(STATIC_TYPE_NAME(ossimSquareFunction));
}
