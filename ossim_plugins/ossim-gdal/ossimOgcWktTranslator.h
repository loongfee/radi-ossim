#ifndef ossimOgcWktTranslator_HEADER
#define ossimOgcWktTranslator_HEADER
#include <ossim/base/ossimCommon.h>
#include <ossim/base/ossimString.h>
#include <map>
#include <string>
class ossimKeywordlist;
class ossimOgcWktTranslator
{
public:
   ossimOgcWktTranslator();
   
   bool toOssimKwl(const ossimString& wktString,
                   ossimKeywordlist& kwl,
                   const char* prefix=NULL)const;
   
   ossimString fromOssimKwl(const ossimKeywordlist& kwl,
                            const char* prefix=NULL)const;
   /*!
    * Returns the empty string if the datum is not found
    *
    */
   ossimString wktToOssimDatum(const ossimString& datum)const;
   ossimString ossimToWktDatum(const ossimString& datum)const;
   ossimString wktToOssimProjection(const ossimString& projection)const;
   ossimString ossimToWktProjection(const ossimString& projection)const;
   
protected:
   std::map<std::string, std::string> theWktToOssimDatumTranslation;
   std::map<std::string, std::string> theWktToOssimProjectionTranslation;
   std::map<std::string, std::string> theOssimToWktDatumTranslation;
   std::map<std::string, std::string> theOssimToWktProjectionTranslation;
   void initializeDatumTable();
   void initializeProjectionTable();
   
};
#endif
