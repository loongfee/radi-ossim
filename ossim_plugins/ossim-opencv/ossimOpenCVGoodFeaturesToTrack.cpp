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
#include "ossimOpenCVGoodFeaturesToTrack.h"
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

RTTI_DEF1(ossimOpenCVGoodFeaturesToTrack, "ossimOpenCVGoodFeaturesToTrack", ossimImageSourceFilter)

ossimOpenCVGoodFeaturesToTrack::ossimOpenCVGoodFeaturesToTrack(ossimObject* owner)
   :ossimImageSourceFilter(owner),
    theTile(NULL),
    theQualityLevel(0.1),
    theMinDistance(10.0),
    theBlockSize(3),
    theHarrisFlag(0),
    theHarrisFreeParameter(0.04)
{
}

ossimOpenCVGoodFeaturesToTrack::~ossimOpenCVGoodFeaturesToTrack()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVGoodFeaturesToTrack::getTile(const ossimIrect& tileRect, ossim_uint32 resLevel)
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

void ossimOpenCVGoodFeaturesToTrack::initialize()
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

ossimScalarType ossimOpenCVGoodFeaturesToTrack::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }
   
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVGoodFeaturesToTrack::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return 1;
}

bool ossimOpenCVGoodFeaturesToTrack::saveState(ossimKeywordlist& kwl, const char* prefix)const
{
   ossimImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"quality_level", theQualityLevel, true);
   kwl.add(prefix,"min_distance", theMinDistance, true);   
   kwl.add(prefix,"block_size", theBlockSize, true);      
   kwl.add(prefix,"use_harris", theHarrisFlag, true);      
   kwl.add(prefix,"k", theHarrisFreeParameter, true);      
   
   return true;
}

bool ossimOpenCVGoodFeaturesToTrack::loadState(const ossimKeywordlist& kwl,
                                     const char* prefix)
{
   ossimImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "quality_level");
   if(lookup)
   {
	theQualityLevel = ossimString(lookup).toDouble();
       	printf("Read from spec file. quality_level: %f\n",theQualityLevel);
   }
   lookup = kwl.find(prefix, "min_distance");
   if(lookup)
   {
	theMinDistance = ossimString(lookup).toDouble();
       	printf("Read from spec file. min_distance: %f\n",theMinDistance);
   }  
   lookup = kwl.find(prefix, "block_size");
   if(lookup)
   {
	theBlockSize = ossimString(lookup).toInt();
       	printf("Read from spec file. block_size: %d\n",theBlockSize);
   }    
   lookup = kwl.find(prefix, "use_harris");
   if(lookup)
   {
	int tmp = ossimString(lookup).toInt();
	if (tmp!=0) { 
		theHarrisFlag=1; 
	}
       	printf("Read from spec file. use_harris: %d\n",theHarrisFlag);
   }    
   lookup = kwl.find(prefix, "k");
   if(lookup)
   {
	theHarrisFreeParameter = ossimString(lookup).toDouble();
       	printf("Read from spec file. k: %f\n",theHarrisFreeParameter);
   }      

   return true;
}

void ossimOpenCVGoodFeaturesToTrack::runUcharTransformation(ossimImageData* tile)
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
		theKeyPoints.push_back(ossimDpt(corners[i].x,corners[i].y)+tile->getOrigin());         
		cvCircle(output,cvPointFrom32f(corners[i]),1,cvScalar(0),1);
	}
	cvReleaseImageHeader(&input);
	cvReleaseImageHeader(&output);
	cvReleaseImage(&eig_image);
	cvReleaseImage(&temp);
	//}

	theTile->validate(); 
}

void ossimOpenCVGoodFeaturesToTrack::setProperty(ossimRefPtr<ossimProperty> property)
{
	if(!property) return;
    ossimString name = property->getName();

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
	  ossimImageSourceFilter::setProperty(property);
	}
}

ossimRefPtr<ossimProperty> ossimOpenCVGoodFeaturesToTrack::getProperty(const ossimString& name)const
{
	if(name == "quality_level")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theQualityLevel));
            numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "min_distance")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theMinDistance));
            numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "block_size")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theBlockSize),3,7);
			//TODO: The block size can only be 3,5,7.
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "use_harris")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theHarrisFlag),0,1);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "k")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theHarrisFreeParameter));
			numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
    return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVGoodFeaturesToTrack::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
	ossimImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("quality_level");
	propertyNames.push_back("min_distance");
	propertyNames.push_back("block_size");
	propertyNames.push_back("use_harris");
	propertyNames.push_back("k");
}