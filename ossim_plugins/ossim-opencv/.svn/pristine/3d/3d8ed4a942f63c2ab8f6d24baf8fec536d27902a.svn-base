#ifndef ossimOpenCVErodeFilter_HEADER
#define ossimOpenCVErodeFilter_HEADER

#include "ossim/plugin/ossimSharedObjectBridge.h"
#include "ossim/base/ossimString.h"
#include "ossim/imaging/ossimImageSourceFilter.h"

#include "cv.h"

/*! @brief OpenCV Erode Filter 
 *
 * Erodes an image by using a specific structuring element.
 * @param iterations Number of times erosion is applied
 *
 * The function erodes the source image using the specified structuring element that determines the shape of a pixel neighborhood 
 * over which the minimum is taken:
 *
 *  \f$ \min _{(x',y') \, in \, \texttt{element}}src(x+x',y+y') \f$
 *
 * A 3x3 rectangular structuring element is used for erosion. Erosion can be applied several (iterations) times. 
 *
 */
class ossimOpenCVErodeFilter : public ossimImageSourceFilter
{
public:
   ossimOpenCVErodeFilter(ossimObject* owner=NULL);
   ossimOpenCVErodeFilter(ossimImageSource* inputSource);
   ossimOpenCVErodeFilter(ossimObject* owner, ossimImageSource* inputSource);
   virtual ~ossimOpenCVErodeFilter();
   ossimString getShortName()const
      {
         return ossimString("OpenCVErode");
      }
   
   ossimString getLongName()const
      {
         return ossimString("OpenCV Erosion Filter");
      }
   
   virtual ossimRefPtr<ossimImageData> getTile(const ossimIrect& tileRect, ossim_uint32 resLevel=0);
   
   virtual void initialize();
   
   virtual ossimScalarType getOutputScalarType() const;
   
   ossim_uint32 getNumberOfOutputBands() const;
 
   virtual bool saveState(ossimKeywordlist& kwl,
                          const char* prefix=0)const;
   
   /*!
    * Method to the load (recreate) the state of an object from a keyword
    * list.  Return true if ok or false on error.
    */
   virtual bool loadState(const ossimKeywordlist& kwl,
                          const char* prefix=0);

   /*
   * Methods to expose thresholds for adjustment through the GUI
   */
   virtual void setProperty(ossimRefPtr<ossimProperty> property);
   virtual ossimRefPtr<ossimProperty> getProperty(const ossimString& name)const;
   virtual void getPropertyNames(std::vector<ossimString>& propertyNames)const;


protected:
   //ossimRefPtr<ossimImageData> theBlankTile;
   ossimRefPtr<ossimImageData> theTile; 
   //IplConvKernel* theElement;
   int theIterations;
   void runUcharTransformation(ossimImageData* tile);

TYPE_DATA
};

#endif
