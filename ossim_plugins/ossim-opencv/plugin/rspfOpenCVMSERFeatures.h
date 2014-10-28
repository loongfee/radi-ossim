#ifndef rspfOpenCVMSERFeatures_HEADER
#define rspfOpenCVMSERFeatures_HEADER

#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"
#include "rspf/imaging/rspfImageSourceFilter.h"
#include <rspf/imaging/rspfAnnotationObject.h>

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

class rspfOpenCVMSERFeatures : public rspfImageSourceFilter //rspfImageSourceFilter
{
public:
   rspfOpenCVMSERFeatures(rspfObject* owner=NULL);
 
   virtual ~rspfOpenCVMSERFeatures();

   rspfString getShortName()const
      {
         return rspfString("OpenCV MSER Features");
      }
   rspfString getLongName()const
      {
         return rspfString("OpenCV MSER Features Identifier");
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


   std::vector<std::vector<rspfDpt> > theContours;

protected:
   rspfRefPtr<rspfImageData> theTile;
   void runUcharTransformation(rspfImageData* tile);
   
TYPE_DATA
};

#endif
