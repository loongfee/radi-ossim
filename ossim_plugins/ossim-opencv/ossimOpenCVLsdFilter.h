#ifndef ossimOpenCVLsdFilter_HEADER
#define ossimOpenCVLsdFilter_HEADER

#include "ossim/plugin/ossimSharedObjectBridge.h"
#include "ossim/base/ossimString.h"
#include "ossim/imaging/ossimImageSourceFilter.h"
#include <ossim/imaging/ossimImageDataFactory.h>

#include <cv.h>
#include <highgui.h>

/** @brief OpenCV Lsd Filter 
 **
 ** Implements Canny algorithm for line segment detection.
 ** @param threshold The scale threshold
 **
 ** The function cvCanny finds the edges on the input image image and marks them in the output image edges using the Canny algorithm.
 ** The smallest of threshold1 and threshold2 is used for edge linking, the largest is used to find initial segments of strong edges. 
 **
 **/
class ossimOpenCVLsdFilter : public ossimImageSourceFilter
{
public:
   ossimOpenCVLsdFilter(ossimObject* owner=NULL);
   ossimOpenCVLsdFilter(ossimImageSource* inputSource, double scale, ossimFilename outASCII, int saveImage);
   ossimOpenCVLsdFilter(ossimObject* owner,ossimImageSource* inputSource, double scale, ossimFilename outASCII, int saveImage);
   virtual ~ossimOpenCVLsdFilter();
   ossimString getShortName()const
      {
         return ossimString("OpenCVLsd");
      }
   
   ossimString getLongName()const
      {
         return ossimString("OpenCV Lsd Filter");
      }
   
   virtual ossimRefPtr<ossimImageData> getTile(const ossimIrect& tileRect, ossim_uint32 resLevel=0);
   virtual void initialize();
   virtual ossimScalarType getOutputScalarType() const;
   ossim_uint32 getNumberOfOutputBands() const;
   virtual bool saveState(ossimKeywordlist& kwl, const char* prefix=0)const;
   virtual bool loadState(const ossimKeywordlist& kwl, const char* prefix=0);

   /*
   * Methods to expose thresholds for adjustment through the GUI
   */
   virtual void setProperty(ossimRefPtr<ossimProperty> property);
   virtual ossimRefPtr<ossimProperty> getProperty(const ossimString& name)const;
   virtual void getPropertyNames(std::vector<ossimString>& propertyNames)const;

   bool cvLsd(IplImage* input, IplImage* output, ossimIpt offset, double scale = 0.8);

protected:
   ossimRefPtr<ossimImageData> theTile;
   void runUcharTransformation(ossimImageData* tile, ossimIpt offset);
   double theScale;
   ossimFilename theOutASCII;
   int theSaveImage;
   int m_nCount;
   //std::vector< double[7] > theLinesegments;
   ossimString theLineSegments;

TYPE_DATA
};

#endif
