#ifndef rspfOpenCVSquaresFilter_HEADER
#define rspfOpenCVSquaresFilter_HEADER
#include "rspf/plugin/rspfSharedObjectBridge.h"
#include "rspf/base/rspfString.h"


#include "rspf/imaging/rspfImageSourceFilter.h"
#include <rspf/imaging/rspfAnnotationObject.h>
#include "cv.h"

class rspfOpenCVSquaresFilter : public rspfImageSourceFilter //rspfImageSourceFilter
{
public:
   rspfOpenCVSquaresFilter(rspfObject* owner=NULL);
   rspfOpenCVSquaresFilter(rspfImageSource* inputSource,
                        double c1 = 1.0/3.0,
                        double c2 = 1.0/3.0,
                        double c3 = 1.0/3.0);
   rspfOpenCVSquaresFilter(rspfObject* owner,
                        rspfImageSource* inputSource,
                        double c1 = 1.0/3.0,
                        double c2 = 1.0/3.0,
                        double c3 = 1.0/3.0);
   virtual ~rspfOpenCVSquaresFilter();
   rspfString getShortName()const
      {
         return rspfString("OpenCVSquares");
      }
   
   rspfString getLongName()const
      {
         return rspfString("OpenCV Square Object Detection");
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

   //TODO: Add properties for Squares

   std::vector< rspfAnnotationObject * > 	theAnnotationObjectList;
protected:
   rspfRefPtr<rspfImageData> theBlankTile;
   rspfRefPtr<rspfImageData> theTile;
   void runUcharTransformation(rspfImageData* tile);
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
