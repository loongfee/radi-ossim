#ifndef ossimOpenCVPluginFactory_HEADER
#define ossimOpenCVPluginFactory_HEADER

#include "ossim/imaging/ossimImageSourceFactoryBase.h"

class ossimImageFileWriter;
class ossimKeywordlist;
class ossimImageWriterFactory;

/** @brief Plugin Factory  
  *
 **/
class ossimOpenCVPluginFactory: public ossimImageSourceFactoryBase
{   

public:
    virtual ~ossimOpenCVPluginFactory();
    static ossimOpenCVPluginFactory* instance();
    virtual ossimObject* createObject(const ossimString& name)const;
    virtual ossimObject* createObject(const ossimKeywordlist& kwl,const char* prefix=0)const;
    virtual void getTypeNameList(std::vector<ossimString>& typeList)const;
   
 protected:
    ossimOpenCVPluginFactory();
    ossimOpenCVPluginFactory(const ossimOpenCVPluginFactory&);
    const ossimOpenCVPluginFactory& operator=(ossimOpenCVPluginFactory&);
    static ossimOpenCVPluginFactory* theInstance;

TYPE_DATA
};

#endif
