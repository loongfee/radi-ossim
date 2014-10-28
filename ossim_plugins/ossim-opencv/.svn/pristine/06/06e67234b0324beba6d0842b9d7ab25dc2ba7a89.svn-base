#ifndef ossimOpenCVSmoothFilter_HEADER
#define ossimOpenCVSmoothFilter_HEADER

#include "ossim/plugin/ossimSharedObjectBridge.h"
#include "ossim/base/ossimString.h"
#include "ossim/imaging/ossimImageSourceFilter.h"
#include <ossim/imaging/ossimImageDataFactory.h>

#include <stdlib.h>

#include <cv.h>
#include <highgui.h>

/** @brief OpenCV Smooth Filter 
 **
 ** Smooths the image in one of several ways.
 **
 ** Parameters:	
 **     @param smooth_type Type of the smoothing:  
 **     <ul>
 **	<li> <b>CV_BLUR_NO_SCALE</b> - linear convolution with param1xparam2 box kernel (all 1’s)  </li>
 **	<li> <b>CV_BLUR</b> - linear convolution with param1xparam2 box kernel (all 1’s) with subsequent scaling by 1/param1·param2 </li>
 **	<li> <b>CV_GAUSSIAN</b> - linear convolution with a param1xparam2 Gaussian kernel </li>
 **     <li> <b>CV_MEDIAN</b> - median filter with a param1xparam1 square aperture </li>
 **     <li> <b>CV_BILATERAL</b> - bilateral filter with a param1xparam1 square aperture, color_sigma=param3 and spatial_sigma=param4. 
 **             If param1=0, the aperture square side is set to cvRound(param4*1.5)*2+1.</li>
 **	</ul>
 **     @param param1 The first parameter of the smoothing operation, the aperture width. Must be a positive odd number (1, 3, 5, ...)
 **     @param param2 The second parameter of the smoothing operation, the aperture height. Ignored by CV_MEDIAN and CV_BILATERAL methods. In the case of 
 **                simple scaled/non-scaled and Gaussian blur if param2 is zero, it is set to param1. Otherwise it must be a positive odd number.
 **     @param param3 In the case of a Gaussian parameter this parameter may specify Gaussian standard deviation. 
 **                If it is zero, it is calculated from the kernel size: sigma = 0.3 (n/2 - 1) + 0.8, where n = param1 for horizontal kernel and param2 for 
 **                vertical kernel
 **     @param param4 In case of non-square Gaussian kernel the parameter may be used to specify a different (from param3) sigma in the vertical direction. 
 **   
 **/
class ossimOpenCVSmoothFilter : public ossimImageSourceFilter
{
public:
   ossimOpenCVSmoothFilter(ossimObject* owner=NULL);
   ossimOpenCVSmoothFilter(ossimImageSource* inputSource);
   ossimOpenCVSmoothFilter(ossimObject* owner, ossimImageSource* inputSource);
   virtual ~ossimOpenCVSmoothFilter();
   virtual ossimRefPtr<ossimImageData> getTile(const ossimIrect& tileRect, ossim_uint32 resLevel=0);   
   virtual void initialize();   
   virtual ossimScalarType getOutputScalarType() const;   
   ossim_uint32 getNumberOfOutputBands() const; 
   virtual bool saveState(ossimKeywordlist& kwl, const char* prefix=0)const;
   virtual bool loadState(const ossimKeywordlist& kwl,const char* prefix=0);
   ossimString getShortName()const
      {
         return ossimString("OpenCVSmooth");
      }
   
   ossimString getLongName()const
      {
         return ossimString("OpenCV Smooth Filter");
      }   

   /*
   * Methods to expose thresholds for adjustment through the GUI
   */
   virtual void setProperty(ossimRefPtr<ossimProperty> property);
   virtual ossimRefPtr<ossimProperty> getProperty(const ossimString& name)const;
   virtual void getPropertyNames(std::vector<ossimString>& propertyNames)const;

protected:
   ossimRefPtr<ossimImageData> theTile;
   void runUcharTransformation(ossimImageData* tile);
   int theSmoothType;///<smooth type: CV_BLUR_NO_SCALE, CV_BLUR, CV_GAUSSIAN, CV_MEDIAN, CV_BILATERAL 
   int theParam1;///<aperture width
   int theParam2;///<aperture height
   double theParam3;///<If square Gaussian kernel, the Gaussian standard desviation. If non-square Gaussian kernel, the Gaussian standard desviation in the horizontal direction
   double theParam4;///<If non-square Gaussian kernel, the Gaussian standard desviation in the vertical direction 

private:
	void setSmoothType(const ossimString);
	void getSmoothTypeList(std::vector<ossimString>&) const;
	ossimString getSmoothTypeString()const;

TYPE_DATA
};

#endif






