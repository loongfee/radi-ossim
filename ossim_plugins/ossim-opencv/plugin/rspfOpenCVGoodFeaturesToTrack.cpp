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
#include "rspfOpenCVGoodFeaturesToTrack.h"
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

RTTI_DEF1(rspfOpenCVGoodFeaturesToTrack, "rspfOpenCVGoodFeaturesToTrack", rspfImageSourceFilter)

rspfOpenCVGoodFeaturesToTrack::rspfOpenCVGoodFeaturesToTrack(rspfObject* owner)
   :rspfImageSourceFilter(owner),
    theTile(NULL),
    theQualityLevel(0.1),
    theMinDistance(10.0),
    theBlockSize(3),
    theHarrisFlag(0),
    theHarrisFreeParameter(0.04)
{
}

rspfOpenCVGoodFeaturesToTrack::~rspfOpenCVGoodFeaturesToTrack()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVGoodFeaturesToTrack::getTile(const rspfIrect& tileRect, rspf_uint32 resLevel)
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

void rspfOpenCVGoodFeaturesToTrack::initialize()
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

rspfScalarType rspfOpenCVGoodFeaturesToTrack::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }
   
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVGoodFeaturesToTrack::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return 1;
}

bool rspfOpenCVGoodFeaturesToTrack::saveState(rspfKeywordlist& kwl, const char* prefix)const
{
   rspfImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"quality_level", theQualityLevel, true);
   kwl.add(prefix,"min_distance", theMinDistance, true);   
   kwl.add(prefix,"block_size", theBlockSize, true);      
   kwl.add(prefix,"use_harris", theHarrisFlag, true);      
   kwl.add(prefix,"k", theHarrisFreeParameter, true);      
   
   return true;
}

bool rspfOpenCVGoodFeaturesToTrack::loadState(const rspfKeywordlist& kwl,
                                     const char* prefix)
{
   rspfImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "quality_level");
   if(lookup)
   {
	theQualityLevel = rspfString(lookup).toDouble();
       	printf("Read from spec file. quality_level: %f\n",theQualityLevel);
   }
   lookup = kwl.find(prefix, "min_distance");
   if(lookup)
   {
	theMinDistance = rspfString(lookup).toDouble();
       	printf("Read from spec file. min_distance: %f\n",theMinDistance);
   }  
   lookup = kwl.find(prefix, "block_size");
   if(lookup)
   {
	theBlockSize = rspfString(lookup).toInt();
       	printf("Read from spec file. block_size: %d\n",theBlockSize);
   }    
   lookup = kwl.find(prefix, "use_harris");
   if(lookup)
   {
	int tmp = rspfString(lookup).toInt();
	if (tmp!=0) { 
		theHarrisFlag=1; 
	}
       	printf("Read from spec file. use_harris: %d\n",theHarrisFlag);
   }    
   lookup = kwl.find(prefix, "k");
   if(lookup)
   {
	theHarrisFreeParameter = rspfString(lookup).toDouble();
       	printf("Read from spec file. k: %f\n",theHarrisFreeParameter);
   }      

   return true;
}

void rspfOpenCVGoodFeaturesToTrack::runUcharTransformation(rspfImageData* tile)
{   

	IplImage *input;
	IplImage *output;
	IplImage* eig_image;
	IplImage* temp;

	char* bSrc;
	char* bDst;

	//int nChannels = tile->getNumberOfBands();

	//for(int k=0; k<nChannels; k++) {
	input = cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
	output = cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
	eig_image = cvCreateImage( cvGetSize(input),32,1);
	temp = cvCreateImage(cvGetSize(input),32,1);
	CvPoint2D32f corners[100];
	int cornercount=100;
	bSrc = static_cast<char*>(tile->getBuf(0));
	input->imageData=bSrc;
	bDst = static_cast<char*>(theTile->getBuf());
	output->imageData=bDst;
        cvCopy(input,output);
	cvGoodFeaturesToTrack( input, eig_image, temp, corners, &cornercount, 	 
                               theQualityLevel,theMinDistance,NULL,theBlockSize,theHarrisFlag,theHarrisFreeParameter);	   	
	for (int i=0;i<cornercount;i++){
		theKeyPoints.push_back(rspfDpt(corners[i].x,corners[i].y)+tile->getOrigin());         
		cvCircle(output,cvPointFrom32f(corners[i]),1,cvScalar(0),1);
	}
	cvReleaseImageHeader(&input);
	cvReleaseImageHeader(&output);
	cvReleaseImage(&eig_image);
	cvReleaseImage(&temp);
	//}

	theTile->validate(); 
}

void rspfOpenCVGoodFeaturesToTrack::setProperty(rspfRefPtr<rspfProperty> property)
{
	if(!property) return;
    rspfString name = property->getName();

    if(name == "quality_level")
    {
            theQualityLevel = property->valueToString().toDouble();
    }
	else if(name == "min_distance")
    {
            theMinDistance = property->valueToString().toDouble();
    }
	else if(name == "block_size")
    {
            theBlockSize = property->valueToString().toInt();
    }
	else if(name == "use_harris")
    {
            theHarrisFlag = property->valueToString().toInt();
    }
	else if(name == "k")
    {
            theHarrisFreeParameter = property->valueToString().toDouble();
    }
	else
	{
	  rspfImageSourceFilter::setProperty(property);
	}
}

rspfRefPtr<rspfProperty> rspfOpenCVGoodFeaturesToTrack::getProperty(const rspfString& name)const
{
	if(name == "quality_level")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theQualityLevel));
            numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "min_distance")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theMinDistance));
            numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "block_size")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theBlockSize),3,7);
			//TODO: The block size can only be 3,5,7.
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "use_harris")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theHarrisFlag),0,1);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "k")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theHarrisFreeParameter));
			numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
    return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVGoodFeaturesToTrack::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
	rspfImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("quality_level");
	propertyNames.push_back("min_distance");
	propertyNames.push_back("block_size");
	propertyNames.push_back("use_harris");
	propertyNames.push_back("k");
}