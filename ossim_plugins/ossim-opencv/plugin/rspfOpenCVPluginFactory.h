#ifndef rspfOpenCVPluginFactory_HEADER
#define rspfOpenCVPluginFactory_HEADER

#include "rspf/imaging/rspfImageSourceFactoryBase.h"

class rspfImageFileWriter;
class rspfKeywordlist;
class rspfImageWriterFactory;

/** @brief Plugin Factory  
  *
 **/
class rspfOpenCVPluginFactory: public rspfImageSourceFactoryBase
{   

public:
    virtual ~rspfOpenCVPluginFactory();
    static rspfOpenCVPluginFactory* instance();
    virtual rspfObject* createObject(const rspfString& name)const;
    virtual rspfObject* createObject(const rspfKeywordlist& kwl,const char* prefix=0)const;
    virtual void getTypeNameList(std::vector<rspfString>& typeList)const;
   
 protected:
    rspfOpenCVPluginFactory();
    rspfOpenCVPluginFactory(const rspfOpenCVPluginFactory&);
    const rspfOpenCVPluginFactory& operator=(rspfOpenCVPluginFactory&);
    static rspfOpenCVPluginFactory* theInstance;

TYPE_DATA
};

#endif
