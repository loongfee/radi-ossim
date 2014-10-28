#ifndef ossimOpenCVGoodFeaturesToTrack_HEADER
#define ossimOpenCVGoodFeaturesToTrack_HEADER

#include "ossim/plugin/ossimSharedObjectBridge.h"
#include "ossim/base/ossimString.h"
#include "ossim/imaging/ossimImageSourceFilter.h"
#include <ossim/imaging/ossimAnnotationObject.h>

#include "cv.h"

/**
@brief OpenCV Good Features To Track

Determines strong corners on an image.

It only supports a 8-bit or floating-point 32-bit single-channel image. 

@param quality_level Multiplier for the max/min eigenvalue; specifies the minimal accepted quality of image corners
@param min_distance Limit, specifying the minimum possible distance between the returned corners; Euclidian distance is used
@param block_size Size of the averaging block, passed to the underlying CornerMinEigenVal or CornerHarris used by the function
@param use_harris If nonzero, Harris operator (CornerHarris) is used instead of default CornerMinEigenVal
@param k Free parameter of Harris detector; used only if useHarris!=0 

If useHarris=0, the function finds the corners with big eigenvalues in the image. The function first calculates the minimal eigenvalue for every source image pixel using the CornerMinEigenVal function and stores them in eigImage. Then it performs non-maxima suppression (only the local maxima in 3x3 neighborhood are retained). The next step rejects the corners with the minimal eigenvalue less than qualityLevel·max(eigImage(x,y)). Finally, the function ensures that the distance between any two corners is not smaller than minDistance. The weaker corners (with a smaller min eigenvalue) that are too close to the stronger corners are rejected.

If useHarris=1, the function runs the Harris edge detector on the image. Similarly to CornerMinEigenVal, for each pixel it calculates a 2x2 gradient covariation matrix M over a blockSizexblockSize neighborhood. Then, it stores

\f$ det(M) - k \, trace(M)^2 \f$

to the destination image. Corners in the image can be found as the local maxima of the destination image.

**/

class ossimOpenCVGoodFeaturesToTrack : public ossimImageSourceFilter //ossimImageSourceFilter
{
public:
   ossimOpenCVGoodFeaturesToTrack(ossimObject* owner=NULL);
 
   virtual ~ossimOpenCVGoodFeaturesToTrack();

   ossimString getShortName()const
      {
         return ossimString("OpenCV GoodFeaturesToTrack");
      }
   ossimString getLongName()const
      {
         return ossimString("OpenCV Good Features Identifier");
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
   double theQualityLevel;
   double theMinDistance;
   int theBlockSize;
   int theHarrisFlag;
   double theHarrisFreeParameter;
   void runUcharTransformation(ossimImageData* tile);
   
TYPE_DATA
};

#endif
