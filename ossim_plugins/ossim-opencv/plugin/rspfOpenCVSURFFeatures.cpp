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
#include "rspfOpenCVSURFFeatures.h"
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

RTTI_DEF1(rspfOpenCVSURFFeatures, "rspfOpenCVSURFFeatures", rspfImageSourceFilter)

rspfOpenCVSURFFeatures::rspfOpenCVSURFFeatures(rspfObject* owner)
   :rspfImageSourceFilter(owner),
    theTile(NULL),
    theHessianThreshold(50)
{
}

rspfOpenCVSURFFeatures::~rspfOpenCVSURFFeatures()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVSURFFeatures::getTile(const rspfIrect& tileRect, rspf_uint32 resLevel)
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

void rspfOpenCVSURFFeatures::initialize()
{
   if(theInputConnection)
   {
      theTile = 0;
      
      theTile = new rspfU8ImageData(this,
                                     1,
                                     theInputConnection->getTileWidth(),
                                     theInputConnection->getTileHeight());  
      theTile->initialize();
   }
}

rspfScalarType rspfOpenCVSURFFeatures::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }
   
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVSURFFeatures::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return 1;
}

bool rspfOpenCVSURFFeatures::saveState(rspfKeywordlist& kwl, const char* prefix)const
{
   rspfImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"hessian_threshold", theHessianThreshold, true);
   
   return true;
}

bool rspfOpenCVSURFFeatures::loadState(const rspfKeywordlist& kwl,
                                     const char* prefix)
{
   rspfImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "hessian_threshold");
   if(lookup)
   {
		theHessianThreshold = rspfString(lookup).toDouble();
       	printf("Read from spec file. hessian_threshold: %f\n",theHessianThreshold);
   }

   return true;
}

void rspfOpenCVSURFFeatures::runUcharTransformation(rspfImageData* tile)
{   

	IplImage *input;
	IplImage *output;
	IplImage *temp;

	char* bSrc;
	char* bDst;

	//int nChannels = tile->getNumberOfBands();

	//for(int k=0; k<nChannels; k++) {
	input = cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
	output = cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
	temp = cvCreateImage(cvGetSize(input),32,1);

	CvMemStorage* storage = cvCreateMemStorage(0);
	
	bSrc = static_cast<char*>(tile->getBuf(0));
	input->imageData=bSrc;
	bDst = static_cast<char*>(theTile->getBuf());
	output->imageData=bDst;
    
	CvSeq *imageKeypoints = NULL;
	cvCopy(input,output);
	
	CvSURFParams params = cvSURFParams(theHessianThreshold, 1);

	cvExtractSURF(input,NULL,&imageKeypoints,NULL,storage,params);

	int numKeyPoints = imageKeypoints->total;

	for (int i=0;i<numKeyPoints;i++){
		CvSURFPoint* corner = (CvSURFPoint*)cvGetSeqElem(imageKeypoints,i);
		theKeyPoints.push_back(rspfDpt(corner->pt.x,corner->pt.y)+tile->getOrigin());         
		cvCircle(output,cvPointFrom32f(corner->pt),1,cvScalar(0),1);
	}
	cvReleaseImageHeader(&input);
	cvReleaseImageHeader(&output);
	cvReleaseImage(&temp);
	//}

	theTile->validate(); 
}

void rspfOpenCVSURFFeatures::setProperty(rspfRefPtr<rspfProperty> property)
{
	if(!property) return;
    rspfString name = property->getName();

    if(name == "hessian_threshold")
    {
            theHessianThreshold = property->valueToString().toDouble();
    }
	else
	{
	  rspfImageSourceFilter::setProperty(property);
	}
}

rspfRefPtr<rspfProperty> rspfOpenCVSURFFeatures::getProperty(const rspfString& name)const
{
	if(name == "hessian_threshold")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theHessianThreshold));
            numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	
    return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVSURFFeatures::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
	rspfImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("hessian_threshold");
}