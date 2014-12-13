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
#include <ossim/imaging/ossimU8ImageData.h>
#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimCommon.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/imaging/ossimImageSourceFactoryBase.h>
#include <ossim/imaging/ossimImageSourceFactoryRegistry.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimNumericProperty.h>
#include <ossim/base/ossimStringProperty.h>

#include "ossimOpenCVThresholdFilter.h"

RTTI_DEF1(ossimOpenCVThresholdFilter, "ossimOpenCVThresholdFilter", ossimImageSourceFilter)

ossimOpenCVThresholdFilter::ossimOpenCVThresholdFilter(ossimObject* owner)
:ossimImageSourceFilter(owner),
theTile(NULL),
theThreshold(10.0),
theMaxValue(255.0),
theThresholdType(1)
{
}

ossimOpenCVThresholdFilter::~ossimOpenCVThresholdFilter()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVThresholdFilter::getTile(const ossimIrect& tileRect, ossim_uint32 resLevel)
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


void ossimOpenCVThresholdFilter::initialize()
{
   if(theInputConnection)
   {
      ossimImageSourceFilter::initialize();
      theTile = new ossimU8ImageData(this,
				     theInputConnection->getNumberOfOutputBands(),   
                                     theInputConnection->getTileWidth(),
                                     theInputConnection->getTileHeight());  
      theTile->initialize();
   }
}

ossimScalarType ossimOpenCVThresholdFilter::getOutputScalarType() const
{
	if(!isSourceEnabled())
	{
		return ossimImageSourceFilter::getOutputScalarType();
	}

	return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVThresholdFilter::getNumberOfOutputBands() const
{
	if(!isSourceEnabled())
	{
		return ossimImageSourceFilter::getNumberOfOutputBands();
	}
   	return theInputConnection->getNumberOfOutputBands();
}

bool ossimOpenCVThresholdFilter::saveState(ossimKeywordlist& kwl, const char* prefix)const
{
	ossimImageSourceFilter::saveState(kwl, prefix);

	kwl.add(prefix,"threshold",theThreshold,true);
	kwl.add(prefix,"max_value",theMaxValue,true);
	
	switch(theThresholdType) {
		case 0:
		   kwl.add(prefix,"threshold_type","CV_THRESH_BINARY",true);
		break;
		case 1:
		   kwl.add(prefix,"threshold_type","CV_THRESH_BINARY_INV",true);
		break;
		case 2:
		   kwl.add(prefix,"threshold_type","CV_THRESH_TRUNC",true);
		break;
		case 3:
		   kwl.add(prefix,"threshold_type","CV_THRESH_TOZERO",true);
		break;
		case 4:
		   kwl.add(prefix,"threshold_type","CV_THRESH_TOZERO_INV",true);
		break;
		case 7:
		   kwl.add(prefix,"threshold_type","CV_THRESH_MASK",true);
		break;
		case 8:
		   kwl.add(prefix,"threshold_type","CV_THRESH_OTSU",true);
		break;
   	}

	return true;
}

void ossimOpenCVThresholdFilter::setThresholdtype(const ossimString& lookup)
{
	if(lookup)
	{
		if(strcmp(lookup,"CV_THRESH_BINARY")==0){
			theThresholdType=0; 
		}
		else if(strcmp(lookup,"CV_THRESH_BINARY_INV")==0){
			theThresholdType=1; 		
		}
		else if(strcmp(lookup,"CV_THRESH_TRUNC")==0){
			theThresholdType=2; 		
		}
		else if(strcmp(lookup,"CV_THRESH_TOZERO")==0){
			theThresholdType=3; 		
		}
		else if(strcmp(lookup,"CV_THRESH_TOZERO_INV")==0){
			theThresholdType=4; 		
		}
		else if(strcmp(lookup,"CV_THRESH_MASK")==0){
			theThresholdType=7; 		
		}
		else if(strcmp(lookup,"CV_THRESH_OTSU")==0){
			theThresholdType=8; 		
		}
		else {
                        printf("%s not supported as threshold_type parameter for OpenCVThresholdFilter!\nDefault threshold_type: CV_THRESH_BINARY\n",lookup.c_str());
		}
	}
}

bool ossimOpenCVThresholdFilter::loadState(const ossimKeywordlist& kwl, const char* prefix)
{
	ossimImageSourceFilter::loadState(kwl, prefix);

	const char* lookup = kwl.find(prefix, "threshold");
	if(lookup)
	{
		theThreshold = ossimString(lookup).toDouble();
 		printf("Read from spec file. threshold: %f\n",theThreshold);
	}
	lookup = kwl.find(prefix, "max_value");
	if(lookup)
	{
		theMaxValue = ossimString(lookup).toDouble();
 		printf("Read from spec file. max_value: %f\n",theMaxValue);
	}
	lookup = kwl.find(prefix, "threshold_type");
	printf("Read from spec file. threshold_type: %s\n",lookup);
	setThresholdtype(lookup);
  
	return true;
}

void ossimOpenCVThresholdFilter::runUcharTransformation(ossimImageData* tile)
{   

	IplImage *input;
	IplImage *output;

	char* bSrc;
	char* bDst;

	int nChannels = tile->getNumberOfBands();

	for(int k=0; k<nChannels; k++) {
		printf("Channel %d\n",k);
		input=cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
		output=cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
		bSrc = static_cast<char*>(tile->getBuf(k));
		input->imageData=bSrc;
		bDst = static_cast<char*>(theTile->getBuf(k));
		output->imageData=bDst;
		cvThreshold(input,output,theThreshold,theMaxValue,theThresholdType);
		cvReleaseImageHeader(&input);
		cvReleaseImageHeader(&output);
	}

	theTile->validate(); 

}

ossimRefPtr<ossimProperty> ossimOpenCVThresholdFilter::getProperty(const ossimString& name)const
{
   if(name == "threshold")
   {
	   ossimProperty* prop = new ossimNumericProperty("threshold",
		   ossimString::toString(theThreshold));
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "max_value")
   {
	   ossimProperty* prop = new ossimNumericProperty("max_value",
		   ossimString::toString(theMaxValue));
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "threshold_type")
   {
		std::vector<ossimString> constraintList;
		getThresholdTypeList(constraintList);
		ossimString value = getThresholdTypeString();
		ossimProperty* prop = new ossimStringProperty("threshold_type",
														value,
														false,
														constraintList);
		prop->setCacheRefreshBit();
		return prop;
   }
   return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVThresholdFilter::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
        ossimImageSourceFilter::getPropertyNames(propertyNames);
        propertyNames.push_back("threshold");
        propertyNames.push_back("max_value");
        propertyNames.push_back("threshold_type");
}

void ossimOpenCVThresholdFilter::setProperty(ossimRefPtr<ossimProperty> property)
{
        if(!property.valid())
		{
			return;
		} 
        ossimString name = property->getName();

        if(name == "threshold")
        {
                theThreshold = property->valueToString().toDouble();
        }
        else if(name == "max_value")
        {
                theMaxValue = property->valueToString().toDouble();
        }
        else if(name == "threshold_type")
        {
                setThresholdtype(property->valueToString());
        }
		else
		{
		  ossimImageSourceFilter::setProperty(property);
		}
}

void ossimOpenCVThresholdFilter::getThresholdTypeList(
   std::vector<ossimString>& list) const
{
   list.resize(7);

   list[0] = ossimString("CV_THRESH_BINARY");
   list[1] = ossimString("CV_THRESH_BINARY_INV");
   list[2] = ossimString("CV_THRESH_TRUNC");
   list[3] = ossimString("CV_THRESH_TOZERO");
   list[4] = ossimString("CV_THRESH_TOZERO_INV");
   //TODO: THRESH_MASK does not seem to exist any more in latest opencv
   list[5] = ossimString("CV_THRESH_MASK");
   list[6] = ossimString("CV_THRESH_OTSU");
}

ossimString ossimOpenCVThresholdFilter::getThresholdTypeString() const
{
   std::vector<ossimString> list;
   getThresholdTypeList(list);
   int type = theThresholdType>4?(theThresholdType-2):theThresholdType;
   printf("%d Threshold Type",type);
   return list[type];
}
