#ifndef rspfOpenCVStarFeatures_HEADER
#define rspfOpenCVStarFeatures_HEADER

#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"
#include "rspf/imaging/rspfImageSourceFilter.h"
#include <rspf/imaging/rspfAnnotationObject.h>

#include "cv.h"

/**
@brief OpenCV Star based KeyPoints
** (Description from OpenCV Source)
**     
the full constructor initialized all the algorithm parameters:
maxSize - maximum size of the features. The following
values of the parameter are supported:
4, 6, 8, 11, 12, 16, 22, 23, 32, 45, 46, 64, 90, 128
responseThreshold - threshold for the approximated laplacian,
used to eliminate weak features. The larger it is,
the less features will be retrieved
lineThresholdProjected - another threshold for the laplacian to
eliminate edges
lineThresholdBinarized - another threshold for the feature
size to eliminate edges.
The larger the 2 threshold, the more points you get.
**/

class rspfOpenCVStarFeatures : public rspfImageSourceFilter //rspfImageSourceFilter
{
public:
   rspfOpenCVStarFeatures(rspfObject* owner=NULL);
 
   virtual ~rspfOpenCVStarFeatures();

   rspfString getShortName()const
      {
         return rspfString("OpenCV Star Features");
      }
   rspfString getLongName()const
      {
         return rspfString("OpenCV Star Features Identifier");
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
   int theMaxSize;
   int theResponseThreshold;
   int theLineThresholdProj;
   int theLineThresholdBin;
TYPE_DATA
};

#endif
