#ifndef ossimOpenCVStarFeatures_HEADER
#define ossimOpenCVStarFeatures_HEADER

#include "ossim/plugin/ossimSharedObjectBridge.h"
#include "ossim/base/ossimString.h"
#include "ossim/imaging/ossimImageSourceFilter.h"
#include <ossim/imaging/ossimAnnotationObject.h>

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

class ossimOpenCVStarFeatures : public ossimImageSourceFilter //ossimImageSourceFilter
{
public:
   ossimOpenCVStarFeatures(ossimObject* owner=NULL);
 
   virtual ~ossimOpenCVStarFeatures();

   ossimString getShortName()const
      {
         return ossimString("OpenCV Star Features");
      }
   ossimString getLongName()const
      {
         return ossimString("OpenCV Star Features Identifier");
      }
  
   virtual ossimRefPtr<ossimImageData> getTile(const ossimIrect& tileRect, ossim_uint32 resLevel=0);
   
   virtual void initialize();
   
   virtual ossimScalarType getOutputScalarType() const;
   
   ossim_uint32 getNumberOfOutputBands() const;
 
   virtual bool saveState(ossimKeywordlist& kwl, const char* prefix=0)const;
   
   /*!
    * Method to the load (recreate) the state of an object from a keyword
    * list.  Return true if ok or false on error.
    */
   virtual bool loadState(const ossimKeywordlist& kwl, const char* prefix=0);

   /*
   * Methods to expose thresholds for adjustment through the GUI
   */
   virtual void setProperty(ossimRefPtr<ossimProperty> property);
   virtual ossimRefPtr<ossimProperty> getProperty(const ossimString& name)const;
   virtual void getPropertyNames(std::vector<ossimString>& propertyNames)const;


   std::vector<ossimDpt> theKeyPoints;

protected:
   ossimRefPtr<ossimImageData> theTile;
   void runUcharTransformation(ossimImageData* tile);
   int theMaxSize;
   int theResponseThreshold;
   int theLineThresholdProj;
   int theLineThresholdBin;
TYPE_DATA
};

#endif
