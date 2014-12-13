#ifndef ossimOpenCVMSERFeatures_HEADER
#define ossimOpenCVMSERFeatures_HEADER

#include "ossim/plugin/ossimSharedObjectBridge.h"
#include "ossim/base/ossimString.h"
#include "ossim/imaging/ossimImageSourceFilter.h"
#include <ossim/imaging/ossimAnnotationObject.h>

#include "cv.h"

/**
@brief OpenCV MSER based Contours
** (Description from OpenCV Source)
** 
Maximally-Stable Extremal Region Extractor
The class encapsulates all the parameters of MSER
(see http://en.wikipedia.org/wiki/Maximally_stable_extremal_regions )
extraction algorithm.
**/

class ossimOpenCVMSERFeatures : public ossimImageSourceFilter //ossimImageSourceFilter
{
public:
   ossimOpenCVMSERFeatures(ossimObject* owner=NULL);
 
   virtual ~ossimOpenCVMSERFeatures();

   ossimString getShortName()const
      {
         return ossimString("OpenCV MSER Features");
      }
   ossimString getLongName()const
      {
         return ossimString("OpenCV MSER Features Identifier");
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


   std::vector<std::vector<ossimDpt> > theContours;

protected:
   ossimRefPtr<ossimImageData> theTile;
   void runUcharTransformation(ossimImageData* tile);
   
TYPE_DATA
};

#endif
