#ifndef ossimOpenCVPyrSegmentation_HEADER
#define ossimOpenCVPyrSegmentation_HEADER

#include "ossim/plugin/ossimSharedObjectBridge.h"
#include <ossim/base/ossimString.h>
#include "ossim/imaging/ossimImageSourceFilter.h"

#include "cv.h"
#include "highgui.h"

/** 

@brief OpenCV Pyramids Segmentation 
 
Implements image segmentation by pyramids.

@param level Maximum level of the pyramid for the segmentation
@param threshold1 Error threshold for establishing the links
@param threshold2 Error threshold for for the segments clustering
   
The function implements image segmentation by pyramids. The pyramid builds up to the level @level. The links between any pixel a on level i and its candidate father pixel b on the adjacent level are established if  \f$  p(c(a),c(b)) < threshold1 \f$. After the connected components are defined, they are joined into several clusters. Any two segments A and B belong to the same cluster, if \f$ p(c(A),c(B)) < threshold2 \f$. If the input image has only one channel, then \f$ p(c^1,c^2)=|c^1-c^2| \f$. If the input image has three channels (red, green and blue), then

\f$ p(c^1,c^2) = 0.30 (c^1_ r - c^2_ r) + 0.59 (c^1_ g - c^2_ g) + 0.11 (c^1_ b - c^2_ b). \f$ 

There may be more than one connected component per a cluster. The images src and dst should be 8-bit n-channel images or equal size.

**/

class ossimOpenCVPyrSegmentation : public ossimImageSourceFilter
{
public:
   ossimOpenCVPyrSegmentation(ossimObject* owner=NULL);

   virtual ~ossimOpenCVPyrSegmentation();

   ossimString getShortName()const
      {
         return ossimString("OpenCVPyrSegmentation");
      }
   
   ossimString getLongName()const
      {
         return ossimString("OpenCV Pyramid Segmentation");
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

protected:
   ossimRefPtr<ossimImageData> theTile;///< Output tile
   int theLevel; ///< Maximum level of the pyramid for the segmentation
   double theThreshold1; ///< Error threshold for establishing the links
   double theThreshold2; ///< Error threshold for for the segments clustering
   void runUcharTransformation(ossimImageData* tile);

TYPE_DATA
};

#endif
