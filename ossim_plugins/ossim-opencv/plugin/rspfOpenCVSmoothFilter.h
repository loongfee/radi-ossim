#ifndef rspfOpenCVSmoothFilter_HEADER
#define rspfOpenCVSmoothFilter_HEADER

#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"
#include "rspf/imaging/rspfImageSourceFilter.h"
#include <rspf/imaging/rspfImageDataFactory.h>

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
class rspfOpenCVSmoothFilter : public rspfImageSourceFilter
{
public:
   rspfOpenCVSmoothFilter(rspfObject* owner=NULL);
   rspfOpenCVSmoothFilter(rspfImageSource* inputSource);
   rspfOpenCVSmoothFilter(rspfObject* owner, rspfImageSource* inputSource);
   virtual ~rspfOpenCVSmoothFilter();
   virtual rspfRefPtr<rspfImageData> getTile(const rspfIrect& tileRect, rspf_uint32 resLevel=0);   
   virtual void initialize();   
   virtual rspfScalarType getOutputScalarType() const;   
   rspf_uint32 getNumberOfOutputBands() const; 
   virtual bool saveState(rspfKeywordlist& kwl, const char* prefix=0)const;
   virtual bool loadState(const rspfKeywordlist& kwl,const char* prefix=0);
   rspfString getShortName()const
      {
         return rspfString("OpenCVSmooth");
      }
   
   rspfString getLongName()const
      {
         return rspfString("OpenCV Smooth Filter");
      }   

   /*
   * Methods to expose thresholds for adjustment through the GUI
   */
   virtual void setProperty(rspfRefPtr<rspfProperty> property);
   virtual rspfRefPtr<rspfProperty> getProperty(const rspfString& name)const;
   virtual void getPropertyNames(std::vector<rspfString>& propertyNames)const;

protected:
   rspfRefPtr<rspfImageData> theTile;
   void runUcharTransformation(rspfImageData* tile);
   int theSmoothType;///<smooth type: CV_BLUR_NO_SCALE, CV_BLUR, CV_GAUSSIAN, CV_MEDIAN, CV_BILATERAL 
   int theParam1;///<aperture width
   int theParam2;///<aperture height
   double theParam3;///<If square Gaussian kernel, the Gaussian standard desviation. If non-square Gaussian kernel, the Gaussian standard desviation in the horizontal direction
   double theParam4;///<If non-square Gaussian kernel, the Gaussian standard desviation in the vertical direction 

private:
	void setSmoothType(const rspfString);
	void getSmoothTypeList(std::vector<rspfString>&) const;
	rspfString getSmoothTypeString()const;

TYPE_DATA
};

#endif






