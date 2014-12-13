#ifndef rspfOpenCVCannyFilter_HEADER
#define rspfOpenCVCannyFilter_HEADER

#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"
#include "rspf/imaging/rspfImageSourceFilter.h"
#include <rspf/imaging/rspfImageDataFactory.h>

#include <cv.h>
#include <highgui.h>

/** @brief OpenCV Canny Filter 
 **
 ** Implements Canny algorithm for edge detection.
 ** @param threshold1 The first threshold     
 ** @param threshold2 The second threshold
 ** @param aperture_size Aperture parameter for Sobel operator
 **
 ** The function cvCanny finds the edges on the input image image and marks them in the output image edges using the Canny algorithm.
 ** The smallest of threshold1 and threshold2 is used for edge linking, the largest is used to find initial segments of strong edges. 
 **
 **/
class rspfOpenCVCannyFilter : public rspfImageSourceFilter
{
public:
   rspfOpenCVCannyFilter(rspfObject* owner=NULL);
   rspfOpenCVCannyFilter(rspfImageSource* inputSource, double threshold1, double threshold2, int apertureSize);
   rspfOpenCVCannyFilter(rspfObject* owner,rspfImageSource* inputSource, double threshold1, double threshold2, int apertureSize);
   virtual ~rspfOpenCVCannyFilter();
   rspfString getShortName()const
      {
         return rspfString("OpenCVCanny");
      }
   
   rspfString getLongName()const
      {
         return rspfString("OpenCV Canny Filter");
      }
   
   virtual rspfRefPtr<rspfImageData> getTile(const rspfIrect& tileRect, rspf_uint32 resLevel=0);
   virtual void initialize();
   virtual rspfScalarType getOutputScalarType() const;
   rspf_uint32 getNumberOfOutputBands() const;
   virtual bool saveState(rspfKeywordlist& kwl, const char* prefix=0)const;
   virtual bool loadState(const rspfKeywordlist& kwl, const char* prefix=0);

   /*
   * Methods to expose thresholds for adjustment through the GUI
   */
   virtual void setProperty(rspfRefPtr<rspfProperty> property);
   virtual rspfRefPtr<rspfProperty> getProperty(const rspfString& name)const;
   virtual void getPropertyNames(std::vector<rspfString>& propertyNames)const;

protected:
   rspfRefPtr<rspfImageData> theTile;
   void runUcharTransformation(rspfImageData* tile);
   double theThreshold1;
   double theThreshold2;
   int theApertureSize;

TYPE_DATA
};

#endif
