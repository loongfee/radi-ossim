#ifndef rspfOpenCVLaplaceFilter_HEADER
#define rspfOpenCVLaplaceFilter_HEADER

#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"
#include "rspf/imaging/rspfImageSourceFilter.h"

#include <stdlib.h>

#include "cv.h"

/** @brief OpenCV Laplace Filter 
  *
  * Calculates the Laplacian of an image.
  * @param aperture_size Aperture size (it has the same meaning as Sobel)
  *
  * The function calculates the Laplacian of the source image by adding up the second x and y derivatives calculated using the Sobel operator:
  *
  * \f$ \texttt{dst}(x,y) = \frac{d^2 \texttt{src}}{dx^2} + \frac{d^2 \texttt{src}}{dy^2} \f$
  *
  * Setting apertureSize = 1 gives the fastest variant that is equal to convolving the image with the following kernel:
  *
  * \f$  \left[ \begin{array}{ccc}
  *		 0 & 1 & 0  \\
  *		 1 & -4 & 1 \\
  *		 0 & 1 & 0  \\
  *	 \end{array} \right]
  * \f$ 
  *
  * Similar to the Sobel function, no scaling is done and the same combinations of input and output formats are supported.
  *
**/
class rspfOpenCVLaplaceFilter : public rspfImageSourceFilter
{

public:
   rspfOpenCVLaplaceFilter(rspfObject* owner=NULL);
   rspfOpenCVLaplaceFilter(rspfImageSource* inputSource,
                        int aperture_size = 3);
   rspfOpenCVLaplaceFilter(rspfObject* owner,
                        rspfImageSource* inputSource,
                        int aperture_size = 3);
   virtual ~rspfOpenCVLaplaceFilter();
   rspfString getShortName()const
      {
         return rspfString("OpenCVLaplace");
      }
   
   rspfString getLongName()const
      {
         return rspfString("OpenCV Laplace Filter");
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
   rspfRefPtr<rspfImageData> theTile; ///< Output tile
   int theApertureSize; ///< Size of the extended Laplace kernel, must be 1, 3, 5 or 7 
   void runUcharTransformation(rspfImageData* tile); 

private:
	void setApertureSize(const int);

TYPE_DATA
};

#endif
