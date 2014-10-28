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
#include "ossimOpenCVSURFFeatures.h"
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

RTTI_DEF1(ossimOpenCVSURFFeatures, "ossimOpenCVSURFFeatures", ossimImageSourceFilter)

ossimOpenCVSURFFeatures::ossimOpenCVSURFFeatures(ossimObject* owner)
   :ossimImageSourceFilter(owner),
    theTile(NULL),
    theHessianThreshold(50)
{
}

ossimOpenCVSURFFeatures::~ossimOpenCVSURFFeatures()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVSURFFeatures::getTile(const ossimIrect& tileRect, ossim_uint32 resLevel)
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

void ossimOpenCVSURFFeatures::initialize()
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

ossimScalarType ossimOpenCVSURFFeatures::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }
   
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVSURFFeatures::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return 1;
}

bool ossimOpenCVSURFFeatures::saveState(ossimKeywordlist& kwl, const char* prefix)const
{
   ossimImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"hessian_threshold", theHessianThreshold, true);
   
   return true;
}

bool ossimOpenCVSURFFeatures::loadState(const ossimKeywordlist& kwl,
                                     const char* prefix)
{
   ossimImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "hessian_threshold");
   if(lookup)
   {
		theHessianThreshold = ossimString(lookup).toDouble();
       	printf("Read from spec file. hessian_threshold: %f\n",theHessianThreshold);
   }

   return true;
}

void ossimOpenCVSURFFeatures::runUcharTransformation(ossimImageData* tile)
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
		theKeyPoints.push_back(ossimDpt(corner->pt.x,corner->pt.y)+tile->getOrigin());         
		cvCircle(output,cvPointFrom32f(corner->pt),1,cvScalar(0),1);
	}
	cvReleaseImageHeader(&input);
	cvReleaseImageHeader(&output);
	cvReleaseImage(&temp);
	//}

	theTile->validate(); 
}

void ossimOpenCVSURFFeatures::setProperty(ossimRefPtr<ossimProperty> property)
{
	if(!property) return;
    ossimString name = property->getName();

    if(name == "hessian_threshold")
    {
            theHessianThreshold = property->valueToString().toDouble();
    }
	else
	{
	  ossimImageSourceFilter::setProperty(property);
	}
}

ossimRefPtr<ossimProperty> ossimOpenCVSURFFeatures::getProperty(const ossimString& name)const
{
	if(name == "hessian_threshold")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theHessianThreshold));
            numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	
    return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVSURFFeatures::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
	ossimImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("hessian_threshold");
}