// Copyright (C) 2010 Argongra 
//
// RSPF is free software; you can redistribute it and/or
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

#include <rspf/base/rspfRefPtr.h>
#include "rspfOpenCVMSERFeatures.h"
#include <rspf/imaging/rspfU8ImageData.h>
#include <rspf/base/rspfConstants.h>
#include <rspf/base/rspfCommon.h>
#include <rspf/base/rspfKeywordlist.h>
#include <rspf/base/rspfKeywordNames.h>
#include <rspf/imaging/rspfImageSourceFactoryBase.h>
#include <rspf/imaging/rspfImageSourceFactoryRegistry.h>
#include <rspf/imaging/rspfAnnotationPolyObject.h>
#include <rspf/base/rspfRefPtr.h>
#include <rspf/base/rspfNumericProperty.h>

RTTI_DEF1(rspfOpenCVMSERFeatures, "rspfOpenCVMSERFeatures", rspfImageSourceFilter)

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

rspfOpenCVMSERFeatures::rspfOpenCVMSERFeatures(rspfObject* owner)
   :rspfImageSourceFilter(owner),
    theTile(NULL)
{
}

rspfOpenCVMSERFeatures::~rspfOpenCVMSERFeatures()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVMSERFeatures::getTile(const rspfIrect& tileRect, rspf_uint32 resLevel)
{

	if(!isSourceEnabled())
	{
	      return rspfImageSourceFilter::getTile(tileRect, resLevel);
	}
	long w     = tileRect.width();
	long h     = tileRect.height();
	   
	if(!theTile.valid()) initialize();
	if(!theTile.valid()) return 0;
	  
	rspfRefPtr<rspfImageData> data = 0;
	if(theInputConnection)
	{
		data  = theInputConnection->getTile(tileRect, resLevel);
	} else {
	      return 0;
	}

	if(!data.valid()) return 0;
	if(data->getDataObjectStatus() == RSPF_NULL ||  data->getDataObjectStatus() == RSPF_EMPTY)
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

void rspfOpenCVMSERFeatures::initialize()
{
   if(theInputConnection)
   {
      theTile = 0;
      
      theTile = new rspfU8ImageData(this,
                                     3,
                                     theInputConnection->getTileWidth(),
                                     theInputConnection->getTileHeight());  
      theTile->initialize();
   }
}

rspfScalarType rspfOpenCVMSERFeatures::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }
   
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVMSERFeatures::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return 3;
}

bool rspfOpenCVMSERFeatures::saveState(rspfKeywordlist& kwl, const char* prefix)const
{
   rspfImageSourceFilter::saveState(kwl, prefix);   
   return true;
}

bool rspfOpenCVMSERFeatures::loadState(const rspfKeywordlist& kwl,
                                     const char* prefix)
{
   rspfImageSourceFilter::loadState(kwl, prefix);
   return true;
}

void rspfOpenCVMSERFeatures::runUcharTransformation(rspfImageData* tile)
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

void rspfOpenCVMSERFeatures::setProperty(rspfRefPtr<rspfProperty> property)
{
	if(!property) return;
    rspfString name = property->getName();
	rspfImageSourceFilter::setProperty(property);
}

rspfRefPtr<rspfProperty> rspfOpenCVMSERFeatures::getProperty(const rspfString& name)const
{	
    return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVMSERFeatures::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
	rspfImageSourceFilter::getPropertyNames(propertyNames);
}
