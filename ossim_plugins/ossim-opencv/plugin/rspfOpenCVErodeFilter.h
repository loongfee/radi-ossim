#ifndef rspfOpenCVErodeFilter_HEADER
#define rspfOpenCVErodeFilter_HEADER

#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"
#include "rspf/imaging/rspfImageSourceFilter.h"

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
class rspfOpenCVErodeFilter : public rspfImageSourceFilter
{
public:
   rspfOpenCVErodeFilter(rspfObject* owner=NULL);
   rspfOpenCVErodeFilter(rspfImageSource* inputSource);
   rspfOpenCVErodeFilter(rspfObject* owner, rspfImageSource* inputSource);
   virtual ~rspfOpenCVErodeFilter();
   rspfString getShortName()const
      {
         return rspfString("OpenCVErode");
      }
   
   rspfString getLongName()const
      {
         return rspfString("OpenCV Erosion Filter");
      }
   
   virtual rspfRefPtr<rspfImageData> getTile(const rspfIrect& tileRect, rspf_uint32 resLevel=0);
   
   virtual void initialize();
   
   virtual rspfScalarType getOutputScalarType() const;
   
   rspf_uint32 getNumberOfOutputBands() const;
 
   virtual bool saveState(rspfKeywordlist& kwl,
                          const char* prefix=0)const;
   
   /*!
    * Method to the load (recreate) the state of an object from a keyword
    * list.  Return true if ok or false on error.
    */
   virtual bool loadState(const rspfKeywordlist& kwl,
                          const char* prefix=0);

   /*
   * Methods to expose thresholds for adjustment through the GUI
   */
   virtual void setProperty(rspfRefPtr<rspfProperty> property);
   virtual rspfRefPtr<rspfProperty> getProperty(const rspfString& name)const;
   virtual void getPropertyNames(std::vector<rspfString>& propertyNames)const;


protected:
   //rspfRefPtr<rspfImageData> theBlankTile;
   rspfRefPtr<rspfImageData> theTile; 
   //IplConvKernel* theElement;
   int theIterations;
   void runUcharTransformation(rspfImageData* tile);

TYPE_DATA
};

#endif
