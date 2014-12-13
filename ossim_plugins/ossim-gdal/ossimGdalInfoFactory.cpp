#include <ossimGdalInfoFactory.h>
#include <ossimOgrInfo.h>
#include <ossimHdfInfo.h>
#include <ossim/support_data/ossimInfoFactory.h>
#include <ossim/support_data/ossimInfoBase.h>
#include <ossim/base/ossimFilename.h>
#include <ossim/base/ossimRefPtr.h>
ossimGdalInfoFactory::~ossimGdalInfoFactory()
{}
ossimGdalInfoFactory* ossimGdalInfoFactory::instance()
{
   static ossimGdalInfoFactory sharedInstance;
   return &sharedInstance;
}
ossimInfoBase* ossimGdalInfoFactory::create(const ossimFilename& file) const
{
   ossimRefPtr<ossimInfoBase> result = 0;
   result = new ossimHdfInfo();
   if ( result->open(file) )
   {
      return result.release();
   }
   result = new ossimOgrInfo();
   if ( result->open(file) )
   {
      return result.release();
   }
   return 0;
}
ossimGdalInfoFactory::ossimGdalInfoFactory()
{}
ossimGdalInfoFactory::ossimGdalInfoFactory(const ossimGdalInfoFactory& /* obj */ )
{}
const ossimGdalInfoFactory& ossimGdalInfoFactory::operator=(
   const ossimGdalInfoFactory& /* rhs */)
{
   return *this;
}
