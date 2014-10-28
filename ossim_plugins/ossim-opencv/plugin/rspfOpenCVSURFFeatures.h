#ifndef rspfOpenCVSURFFeatures_HEADER
#define rspfOpenCVSURFFeatures_HEADER

#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"
#include "rspf/imaging/rspfImageSourceFilter.h"
#include <rspf/imaging/rspfAnnotationObject.h>

#include "cv.h"

/**
@brief OpenCV SURF based KeyPoints
** (Description from OpenCV Source)
** 
The function cvExtractSURF finds robust features 
in the image, as described in Bay06 . For each feature
it returns its location, size, orientation and optionally
the descriptor, basic or extended. The function can be
used for object tracking and localization, image stitching etc. 
**/

class rspfOpenCVSURFFeatures : public rspfImageSourceFilter //rspfImageSourceFilter
{
public:
   rspfOpenCVSURFFeatures(rspfObject* owner=NULL);
 
   virtual ~rspfOpenCVSURFFeatures();

   rspfString getShortName()const
      {
         return rspfString("OpenCV SURF Features");
      }
   rspfString getLongName()const
      {
         return rspfString("OpenCV SURF Features Identifier");
      }
  
   virtual rspfRefPtr<rspfImageData> getTile(const rspfIrect& tileRect, rspf_uint32 resLevel=0);
   
   virtual void initialize();
   
   virtual rspfScalarType getOutputScalarType() const;
   
   rspf_uint32 getNumberOfOutputBands() const;
 
   virtual bool saveState(rspfKeywordlist& kwl, const char* prefix=0)const;
   
   /*!
    * Method to the load (recreate) the state of an object from a keyword
    * list.  Return true if ok or false on error.
    */
   virtual bool loadState(const rspfKeywordlist& kwl, const char* prefix=0);

   /*
   * Methods to expose thresholds for adjustment through the GUI
   */
   virtual void setProperty(rspfRefPtr<rspfProperty> property);
   virtual rspfRefPtr<rspfProperty> getProperty(const rspfString& name)const;
   virtual void getPropertyNames(std::vector<rspfString>& propertyNames)const;


   std::vector<rspfDpt> theKeyPoints;

protected:
   rspfRefPtr<rspfImageData> theTile;
   void runUcharTransformation(rspfImageData* tile);
   double theHessianThreshold;
   
TYPE_DATA
};

#endif
