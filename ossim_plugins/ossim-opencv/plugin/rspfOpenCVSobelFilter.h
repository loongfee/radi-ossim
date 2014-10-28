#ifndef rspfOpenCVSobelFilter_HEADER
#define rspfOpenCVSobelFilter_HEADER

#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"
#include "rspf/imaging/rspfImageSourceFilter.h"

#include <stdlib.h>

#include "cv.h"

/** @brief OpenCV Sobel Filter 

    Calculates the first, second, third or mixed image derivatives using an extended Sobel operator.
    @param xorder Order of the derivative x
    @param yorder Order of the derivative y
    @param aperture_size Size of the extended Sobel kernel, must be 1, 3, 5 or 7

    In all cases except 1, an \f$ \texttt{apertureSize} \times \texttt{apertureSize} \f$  separable kernel will be used to calculate the derivative. For \f$ \texttt{apertureSize} = 1 \f$  a \f$  3 \times 1 \f$  or \f$ 1 \times 3 \f$  a kernel is used (Gaussian smoothing is not done). There is also the special value CV_SCHARR (-1) that corresponds to a \f$  3\times 3 \f$ Scharr filter that may give more accurate results than a \f$ 3\times 3 \f$  Sobel. Scharr aperture is

    \f$  \vecthreethree {-3}{0}{3} {-10}{0}{10} {-3}{0}{3} \f$ 

    for the x-derivative or transposed for the y-derivative.

    The function calculates the image derivative by convolving the image with the appropriate kernel:

    \f$  \texttt{dst}(x,y) = \frac{d^{xorder+yorder}}{dx^{xorder} \cdot dy^{yorder}} \texttt{src} \f$ 

    The Sobel operators combine Gaussian smoothing and differentiation so the result is more or less resistant to the noise. Most often, the function is called with (xorder = 1, yorder = 0, apertureSize = 3) or (xorder = 0, yorder = 1, apertureSize = 3) to calculate the first x- or y- image derivative. The first case corresponds to a kernel of:

    \f$  \left[ \begin{array}{ccc}
		-1 & 0 & 1 \\
		-2 & 0 & 2 \\
		 1 & 2 & 1 \\
	 \end{array} \right]
    \f$ 

    and the second one corresponds to a kernel of:

    \f$  \left[ \begin{array}{ccc}
		-1 & -2 & -1 \\
		 0 & 0 & 0 \\
		 1 & 2 & 1 \\
	 \end{array} \right]
    \f$ 

    or a kernel of:

    \f$  \left[ \begin{array}{ccc}
		 1 & 2 & 1 \\
		 0 & 0 & 0 \\
		 -1 & -2 & -1 \\
	 \end{array} \right]
    \f$ 

    depending on the image origin (origin field of IplImage structure). Besides 8-bit images the function can process 32-bit floating-point images. Both the source and the destination must be single-channel images of equal size or equal ROI size.

**/

class rspfOpenCVSobelFilter : public rspfImageSourceFilter
{
public:
   rspfOpenCVSobelFilter(rspfObject* owner=NULL);
   rspfOpenCVSobelFilter(rspfImageSource* inputSource,
                        int xorder = 1 ,
                        int yorder = 1,
                        int aperture_size = 3);
   rspfOpenCVSobelFilter(rspfObject* owner,
                        rspfImageSource* inputSource,
                        int xorder = 1 ,
                        int yorder = 1,
                        int aperture_size = 3);
   virtual ~rspfOpenCVSobelFilter();
   rspfString getShortName()const
      {
         return rspfString("OpenCVSobel");
      }
   
   rspfString getLongName()const
      {
         return rspfString("OpenCV Sobel Filter");
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
   int theXOrder; ///< Order of the derivative of x
   int theYOrder; ///< Order of the derivative of y
   int theApertureSize; ///< Size of the extended Soble kernel, must be 1, 3, 5 or 7 
   void runUcharTransformation(rspfImageData* tile); 

private:
	void setApertureSize(const rspfString);

TYPE_DATA
};

#endif
