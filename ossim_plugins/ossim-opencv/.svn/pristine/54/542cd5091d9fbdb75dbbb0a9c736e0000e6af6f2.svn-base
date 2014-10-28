// Copyright (C) 2010 Argongra 
//
// OSSIM is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License 
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
//
// You should have received a copy of the GNU General Public License
// along with this software. If not, write to the Free Software 
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-
// 1307, USA.
//
// See the GPL in the COPYING.GPL file for more details.
//
//*************************************************************************

#include <ossim/base/ossimRefPtr.h>
#include "ossimOpenCVMSERFeatures.h"
#include <ossim/imaging/ossimU8ImageData.h>
#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimCommon.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/imaging/ossimImageSourceFactoryBase.h>
#include <ossim/imaging/ossimImageSourceFactoryRegistry.h>
#include <ossim/imaging/ossimAnnotationPolyObject.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimNumericProperty.h>

RTTI_DEF1(ossimOpenCVMSERFeatures, "ossimOpenCVMSERFeatures", ossimImageSourceFilter)

static CvScalar colors[] = 
   {
	   {{0,0,255}},
	   {{0,128,255}},
	   {{0,255,255}},
	   {{0,255,0}},
	   {{255,128,0}},
	   {{255,255,0}},
	   {{255,0,0}},
	   {{255,0,255}},
	   {{255,255,255}},
	   {{196,255,255}},
	   {{255,255,196}}
   };

   static uchar bcolors[][3] = 
   {
	   {0,0,255},
	   {0,128,255},
	   {0,255,255},
	   {0,255,0},
	   {255,128,0},
	   {255,255,0},
	   {255,0,0},
	   {255,0,255},
	   {255,255,255}
   };

ossimOpenCVMSERFeatures::ossimOpenCVMSERFeatures(ossimObject* owner)
   :ossimImageSourceFilter(owner),
    theTile(NULL)
{
}

ossimOpenCVMSERFeatures::~ossimOpenCVMSERFeatures()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVMSERFeatures::getTile(const ossimIrect& tileRect, ossim_uint32 resLevel)
{

	if(!isSourceEnabled())
	{
	      return ossimImageSourceFilter::getTile(tileRect, resLevel);
	}
	long w     = tileRect.width();
	long h     = tileRect.height();
	   
	if(!theTile.valid()) initialize();
	if(!theTile.valid()) return 0;
	  
	ossimRefPtr<ossimImageData> data = 0;
	if(theInputConnection)
	{
		data  = theInputConnection->getTile(tileRect, resLevel);
	} else {
	      return 0;
	}

	if(!data.valid()) return 0;
	if(data->getDataObjectStatus() == OSSIM_NULL ||  data->getDataObjectStatus() == OSSIM_EMPTY)
	{
	     return 0;
	}

	theTile->setImageRectangle(tileRect);
	theTile->makeBlank();
	   
	theTile->setOrigin(tileRect.ul());
	runUcharTransformation(data.get());
	   
	printf("Tile (%d,%d) finished!\n",tileRect.ul().x,tileRect.ul().y); 	

	return theTile;
   
}

void ossimOpenCVMSERFeatures::initialize()
{
   if(theInputConnection)
   {
      theTile = 0;
      
      theTile = new ossimU8ImageData(this,
                                     3,
                                     theInputConnection->getTileWidth(),
                                     theInputConnection->getTileHeight());  
      theTile->initialize();
   }
}

ossimScalarType ossimOpenCVMSERFeatures::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }
   
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVMSERFeatures::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return 3;
}

bool ossimOpenCVMSERFeatures::saveState(ossimKeywordlist& kwl, const char* prefix)const
{
   ossimImageSourceFilter::saveState(kwl, prefix);   
   return true;
}

bool ossimOpenCVMSERFeatures::loadState(const ossimKeywordlist& kwl,
                                     const char* prefix)
{
   ossimImageSourceFilter::loadState(kwl, prefix);
   return true;
}

void ossimOpenCVMSERFeatures::runUcharTransformation(ossimImageData* tile)
{   

	IplImage *input;
	IplImage *output;

	char* bSrc;
	char* bDst;

	CvMemStorage* storage = cvCreateMemStorage(0);

	input = cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
	output = cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,3);
	
	bSrc = static_cast<char*>(tile->getBuf(0));
	input->imageData=bSrc;
	bDst = static_cast<char*>(theTile->getBuf());
	output->imageData=bDst;

	IplImage* hsv = cvCreateImage( cvGetSize( input ), IPL_DEPTH_8U, 3 );

	cvCvtColor(input,output,CV_GRAY2BGR);
	cvCvtColor(output,hsv,CV_BGR2YCrCb );
    
	CvSeq *contours = NULL;
	
	//MSER Usage sample from OpenCV source
	CvMSERParams params = cvMSERParams();//cvMSERParams( 5, 60, cvRound(.2*img->width*img->height), .25, .2 );
	cvExtractMSER( hsv, NULL, &contours, storage, params );
        //t = cvGetTickCount() - t;

	uchar* rsptr = (uchar*)input->imageData;
	// draw mser with different color
	for ( int i = contours->total-1; i >= 0; i-- )
	{
		CvSeq* r = *(CvSeq**)cvGetSeqElem( contours, i );
		for ( int j = 0; j < r->total; j++ )
		{
			CvPoint* pt = CV_GET_SEQ_ELEM( CvPoint, r, j );
			rsptr[pt->x*3+pt->y*input->widthStep] = bcolors[i%9][2];
			rsptr[pt->x*3+1+pt->y*input->widthStep] = bcolors[i%9][1];
			rsptr[pt->x*3+2+pt->y*input->widthStep] = bcolors[i%9][0];
		}
	}
	// find ellipse ( it seems cvfitellipse2 have error or sth?
	//TODO: Use simpler representation of points
	for ( int i = 0; i < contours->total; i++ )
	{
		CvContour* r = *(CvContour**)cvGetSeqElem( contours, i );
		CvBox2D box = cvFitEllipse2( r );
		box.angle=(float)CV_PI/2-box.angle;

		if ( r->color > 0 )
			cvEllipseBox( output, box, colors[9], 2 );
		else
			cvEllipseBox( output, box, colors[2], 2 );

	}

	cvReleaseImageHeader(&input);
	cvReleaseImageHeader(&output);
	cvReleaseImage(&hsv);
	//}

	theTile->validate(); 
}

void ossimOpenCVMSERFeatures::setProperty(ossimRefPtr<ossimProperty> property)
{
	if(!property) return;
    ossimString name = property->getName();
	ossimImageSourceFilter::setProperty(property);
}

ossimRefPtr<ossimProperty> ossimOpenCVMSERFeatures::getProperty(const ossimString& name)const
{	
    return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVMSERFeatures::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
	ossimImageSourceFilter::getPropertyNames(propertyNames);
}
