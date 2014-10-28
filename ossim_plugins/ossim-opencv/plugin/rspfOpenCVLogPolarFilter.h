#ifndef rspfOpenCVLogPolarFilter_HEADER
#define rspfOpenCVLogPolarFilter_HEADER
#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"


#include "rspf/imaging/rspfImageSourceFilter.h"

#include "cv.h"

class rspfOpenCVLogPolarFilter : public rspfImageSourceFilter
{
public:
   rspfOpenCVLogPolarFilter(rspfObject* owner=NULL);
   rspfOpenCVLogPolarFilter(rspfImageSource* inputSource);
   rspfOpenCVLogPolarFilter(rspfObject* owner,
                        rspfImageSource* inputSource);
   virtual ~rspfOpenCVLogPolarFilter();
   rspfString getShortName()const
      {
         return rspfString("OpenCVLogPolar");
      }
   
   rspfString getLongName()const
      {
         return rspfString("OpenCV LogPolar Filter");
      }
   
   virtual rspfRefPtr<rspfImageData> getTile(const rspfIrect& tileRect, rspf_uint32 resLevel=0);
   
   virtual void initialize();
   
   virtual rspfScalarType getOutputScalarType() const;
   
   rspf_uint32 getNumberOfOutputBands() const;
 
   virtual bool saveState(rspfKeywordlist& kwl,
                          const char* prefix=0)const;
   
   /*!
    * Method to the load (recreate) the state of an object from a keyword
    * list.  Return true if ok or false on error.
    */
   virtual bool loadState(const rspfKeywordlist& kwl,
                          const char* prefix=0);

   /*
   * Methods to expose thresholds for adjustment through the GUI
   */
   virtual void setProperty(rspfRefPtr<rspfProperty> property);
   virtual rspfRefPtr<rspfProperty> getProperty(const rspfString& name)const;
   virtual void getPropertyNames(std::vector<rspfString>& propertyNames)const;


protected:
   rspfRefPtr<rspfImageData> theBlankTile;
   rspfRefPtr<rspfImageData> theTile;
   void runUcharTransformation(rspfImageData* tile);
   double thecenter_x;
   double thecenter_y;
   double theM;

TYPE_DATA
};

#endif
