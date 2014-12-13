#ifndef rspfOpenCVDilateFilter_HEADER
#define rspfOpenCVDilateFilter_HEADER

#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"
#include "rspf/imaging/rspfImageSourceFilter.h"

#include "cv.h"

/*! @brief OpenCV Dilate Filter 
 *
 * Dilates an image by using a specific structuring element.
 * @param iterations Number of times dilation is applied
 *
 * The function dilates the source image using the specified structuring element that determines the shape of a pixel neighborhood 
 * over which the minimum is taken:
 *
 *  \f$ \max _{(x',y') \, in \, \texttt{element}}src(x+x',y+y') \f$
 *
 * A 3x3 rectangular structuring element is used for erosion. Dilation can be applied several (iterations) times. 
 *
 */
class rspfOpenCVDilateFilter : public rspfImageSourceFilter
{
public:
   rspfOpenCVDilateFilter(rspfObject* owner=NULL);
   rspfOpenCVDilateFilter(rspfImageSource* inputSource);
   rspfOpenCVDilateFilter(rspfObject* owner,
                        rspfImageSource* inputSource);
   virtual ~rspfOpenCVDilateFilter();
   rspfString getShortName()const
      {
         return rspfString("OpenCVDilate");
      }
   
   rspfString getLongName()const
      {
         return rspfString("OpenCV Dilate Filter");
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
   void runUcharTransformation(rspfImageData* tile);
   int theIterations;

TYPE_DATA
};

#endif
