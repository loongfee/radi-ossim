#ifndef ossimOpenCVSquaresFilter_HEADER
#define ossimOpenCVSquaresFilter_HEADER
#include "ossim/plugin/ossimSharedObjectBridge.h"
#include "ossim/base/ossimString.h"


#include "ossim/imaging/ossimImageSourceFilter.h"
#include <ossim/imaging/ossimAnnotationObject.h>
#include "cv.h"

class ossimOpenCVSquaresFilter : public ossimImageSourceFilter //ossimImageSourceFilter
{
public:
   ossimOpenCVSquaresFilter(ossimObject* owner=NULL);
   ossimOpenCVSquaresFilter(ossimImageSource* inputSource,
                        double c1 = 1.0/3.0,
                        double c2 = 1.0/3.0,
                        double c3 = 1.0/3.0);
   ossimOpenCVSquaresFilter(ossimObject* owner,
                        ossimImageSource* inputSource,
                        double c1 = 1.0/3.0,
                        double c2 = 1.0/3.0,
                        double c3 = 1.0/3.0);
   virtual ~ossimOpenCVSquaresFilter();
   ossimString getShortName()const
      {
         return ossimString("OpenCVSquares");
      }
   
   ossimString getLongName()const
      {
         return ossimString("OpenCV Square Object Detection");
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

   //TODO: Add properties for Squares

   std::vector< ossimAnnotationObject * > 	theAnnotationObjectList;
protected:
   ossimRefPtr<ossimImageData> theBlankTile;
   ossimRefPtr<ossimImageData> theTile;
   void runUcharTransformation(ossimImageData* tile);
   double theC1;
   double theC2;
   double theC3;
   CvMemStorage* storage ;
   int thresh ;
   CvSeq* findSquares4( IplImage* img, CvMemStorage* storage );
void drawSquares( IplImage* img, CvSeq* squares );
double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 );
TYPE_DATA
};

#endif
