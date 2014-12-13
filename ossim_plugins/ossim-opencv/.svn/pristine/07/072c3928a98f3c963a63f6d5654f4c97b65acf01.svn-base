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
#include "ossimOpenCVStarFeatures.h"
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

RTTI_DEF1(ossimOpenCVStarFeatures, "ossimOpenCVStarFeatures", ossimImageSourceFilter)

ossimOpenCVStarFeatures::ossimOpenCVStarFeatures(ossimObject* owner)
   :ossimImageSourceFilter(owner),
    theTile(NULL),
	theMaxSize(45),
	theResponseThreshold(30),
    theLineThresholdProj(10),
    theLineThresholdBin(8)
{
}

ossimOpenCVStarFeatures::~ossimOpenCVStarFeatures()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVStarFeatures::getTile(const ossimIrect& tileRect, ossim_uint32 resLevel)
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

void ossimOpenCVStarFeatures::initialize()
{
   if(theInputConnection)
   {
      theTile = 0;
      
      theTile = new ossimU8ImageData(this,
                                     1,
                                     theInputConnection->getTileWidth(),
                                     theInputConnection->getTileHeight());  
      theTile->initialize();
   }
}

ossimScalarType ossimOpenCVStarFeatures::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }
   
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVStarFeatures::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return 1;
}

bool ossimOpenCVStarFeatures::saveState(ossimKeywordlist& kwl, const char* prefix)const
{
   ossimImageSourceFilter::saveState(kwl, prefix);
   
   kwl.add(prefix,"linear_threshold_projected", theLineThresholdProj, true);
   kwl.add(prefix,"linear_threshold_binarized", theLineThresholdBin, true);
   kwl.add(prefix,"response_threshold", theResponseThreshold, true);

   return true;
}

bool ossimOpenCVStarFeatures::loadState(const ossimKeywordlist& kwl,
                                     const char* prefix)
{
   ossimImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "linear_threshold_projected");
   if(lookup)
   {
		theLineThresholdProj = ossimString(lookup).toInt();
       	printf("Read from spec file. linear_threshold_proj: %d\n",theLineThresholdProj);
   }
   lookup = kwl.find(prefix, "linear_threshold_binarized");
   if(lookup)
   {
		theLineThresholdBin = ossimString(lookup).toInt();
       	printf("Read from spec file. linear_threshold_binarized: %d\n",theLineThresholdBin);
   }
   lookup = kwl.find(prefix, "response_threshold");
   if(lookup)
   {
		theResponseThreshold = ossimString(lookup).toInt();
       	printf("Read from spec file. response_threshold: %d\n",theResponseThreshold);
   }
   return true;
}

void ossimOpenCVStarFeatures::runUcharTransformation(ossimImageData* tile)
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
		theKeyPoints.push_back(ossimDpt(corner->pt.x,corner->pt.y)+tile->getOrigin());         
		cvCircle(output,corner->pt,1,cvScalar(0),1);
	}
	cvReleaseImageHeader(&input);
	cvReleaseImageHeader(&output);
	//}

	theTile->validate(); 
}

void ossimOpenCVStarFeatures::setProperty(ossimRefPtr<ossimProperty> property)
{
	if(!property) return;
    ossimString name = property->getName();
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
	  ossimImageSourceFilter::setProperty(property);
	}
}

ossimRefPtr<ossimProperty> ossimOpenCVStarFeatures::getProperty(const ossimString& name)const
{
	if(name == "linear_threshold_projected")
    {
        ossimNumericProperty* numeric = new ossimNumericProperty(name,
			ossimString::toString(theLineThresholdProj));
        numeric->setCacheRefreshBit();
        return numeric;
    }
	else if(name == "linear_threshold_binarized")
    {
        ossimNumericProperty* numeric = new ossimNumericProperty(name,
			ossimString::toString(theLineThresholdBin));
        numeric->setCacheRefreshBit();
        return numeric;
    }
	else if(name == "response_threshold")
    {
        ossimNumericProperty* numeric = new ossimNumericProperty(name,
			ossimString::toString(theResponseThreshold));
        numeric->setCacheRefreshBit();
        return numeric;
    }
    return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVStarFeatures::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
	ossimImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("linear_threshold_projected");
	propertyNames.push_back("linear_threshold_binarized");
	propertyNames.push_back("response_threshold");
}