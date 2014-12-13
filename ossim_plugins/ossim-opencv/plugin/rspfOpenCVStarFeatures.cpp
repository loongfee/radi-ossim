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
#include "rspfOpenCVStarFeatures.h"
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

RTTI_DEF1(rspfOpenCVStarFeatures, "rspfOpenCVStarFeatures", rspfImageSourceFilter)

rspfOpenCVStarFeatures::rspfOpenCVStarFeatures(rspfObject* owner)
   :rspfImageSourceFilter(owner),
    theTile(NULL),
	theMaxSize(45),
	theResponseThreshold(30),
    theLineThresholdProj(10),
    theLineThresholdBin(8)
{
}

rspfOpenCVStarFeatures::~rspfOpenCVStarFeatures()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVStarFeatures::getTile(const rspfIrect& tileRect, rspf_uint32 resLevel)
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

void rspfOpenCVStarFeatures::initialize()
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

rspfScalarType rspfOpenCVStarFeatures::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }
   
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVStarFeatures::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return 1;
}

bool rspfOpenCVStarFeatures::saveState(rspfKeywordlist& kwl, const char* prefix)const
{
   rspfImageSourceFilter::saveState(kwl, prefix);
   
   kwl.add(prefix,"linear_threshold_projected", theLineThresholdProj, true);
   kwl.add(prefix,"linear_threshold_binarized", theLineThresholdBin, true);
   kwl.add(prefix,"response_threshold", theResponseThreshold, true);

   return true;
}

bool rspfOpenCVStarFeatures::loadState(const rspfKeywordlist& kwl,
                                     const char* prefix)
{
   rspfImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "linear_threshold_projected");
   if(lookup)
   {
		theLineThresholdProj = rspfString(lookup).toInt();
       	printf("Read from spec file. linear_threshold_proj: %d\n",theLineThresholdProj);
   }
   lookup = kwl.find(prefix, "linear_threshold_binarized");
   if(lookup)
   {
		theLineThresholdBin = rspfString(lookup).toInt();
       	printf("Read from spec file. linear_threshold_binarized: %d\n",theLineThresholdBin);
   }
   lookup = kwl.find(prefix, "response_threshold");
   if(lookup)
   {
		theResponseThreshold = rspfString(lookup).toInt();
       	printf("Read from spec file. response_threshold: %d\n",theResponseThreshold);
   }
   return true;
}

void rspfOpenCVStarFeatures::runUcharTransformation(rspfImageData* tile)
{   

	IplImage *input;
	IplImage *output;

	char* bSrc;
	char* bDst;

	//int nChannels = tile->getNumberOfBands();

	//for(int k=0; k<nChannels; k++) {
	input = cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
	output = cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);

	CvMemStorage* storage = cvCreateMemStorage(0);
	
	bSrc = static_cast<char*>(tile->getBuf(0));
	input->imageData=bSrc;
	bDst = static_cast<char*>(theTile->getBuf());
	output->imageData=bDst;
    
	cvCopy(input,output);
	
	CvStarDetectorParams params = cvStarDetectorParams(theMaxSize,theResponseThreshold,
		theLineThresholdProj,theLineThresholdBin);
	
	CvSeq *imageKeypoints = cvGetStarKeypoints(input,storage,params);

	int numKeyPoints = imageKeypoints->total;

	printf("Found %d keypoints\n",numKeyPoints);

	for (int i=0;i<numKeyPoints;i++){
		CvStarKeypoint* corner = (CvStarKeypoint*)cvGetSeqElem(imageKeypoints,i);
		theKeyPoints.push_back(rspfDpt(corner->pt.x,corner->pt.y)+tile->getOrigin());         
		cvCircle(output,corner->pt,1,cvScalar(0),1);
	}
	cvReleaseImageHeader(&input);
	cvReleaseImageHeader(&output);
	//}

	theTile->validate(); 
}

void rspfOpenCVStarFeatures::setProperty(rspfRefPtr<rspfProperty> property)
{
	if(!property) return;
    rspfString name = property->getName();
	if(name == "linear_threshold_projected")
    {
		theLineThresholdProj = property->valueToString().toInt();
    }
	else if(name == "linear_threshold_binarized")
    {
		theLineThresholdBin = property->valueToString().toInt();
    }
	else if(name == "response_threshold")
	{
		theResponseThreshold = property->valueToString().toInt();
	}
	else
	{
	  rspfImageSourceFilter::setProperty(property);
	}
}

rspfRefPtr<rspfProperty> rspfOpenCVStarFeatures::getProperty(const rspfString& name)const
{
	if(name == "linear_threshold_projected")
    {
        rspfNumericProperty* numeric = new rspfNumericProperty(name,
			rspfString::toString(theLineThresholdProj));
        numeric->setCacheRefreshBit();
        return numeric;
    }
	else if(name == "linear_threshold_binarized")
    {
        rspfNumericProperty* numeric = new rspfNumericProperty(name,
			rspfString::toString(theLineThresholdBin));
        numeric->setCacheRefreshBit();
        return numeric;
    }
	else if(name == "response_threshold")
    {
        rspfNumericProperty* numeric = new rspfNumericProperty(name,
			rspfString::toString(theResponseThreshold));
        numeric->setCacheRefreshBit();
        return numeric;
    }
    return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVStarFeatures::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
	rspfImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("linear_threshold_projected");
	propertyNames.push_back("linear_threshold_binarized");
	propertyNames.push_back("response_threshold");
}