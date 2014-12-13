#ifndef ossimOpenCVLaplaceFilter_HEADER
#define ossimOpenCVLaplaceFilter_HEADER

#include "ossim/plugin/ossimSharedObjectBridge.h"
#include "ossim/base/ossimString.h"
#include "ossim/imaging/ossimImageSourceFilter.h"

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
class ossimOpenCVLaplaceFilter : public ossimImageSourceFilter
{

public:
   ossimOpenCVLaplaceFilter(ossimObject* owner=NULL);
   ossimOpenCVLaplaceFilter(ossimImageSource* inputSource,
                        int aperture_size = 3);
   ossimOpenCVLaplaceFilter(ossimObject* owner,
                        ossimImageSource* inputSource,
                        int aperture_size = 3);
   virtual ~ossimOpenCVLaplaceFilter();
   ossimString getShortName()const
      {
         return ossimString("OpenCVLaplace");
      }
   
   ossimString getLongName()const
      {
         return ossimString("OpenCV Laplace Filter");
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
   ossimRefPtr<ossimImageData> theTile; ///< Output tile
   int theApertureSize; ///< Size of the extended Laplace kernel, must be 1, 3, 5 or 7 
   void runUcharTransformation(ossimImageData* tile); 

private:
	void setApertureSize(const int);

TYPE_DATA
};

#endif
