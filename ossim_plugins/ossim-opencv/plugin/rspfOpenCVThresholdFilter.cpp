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
#include <rspf/imaging/rspfU8ImageData.h>
#include <rspf/base/rspfConstants.h>
#include <rspf/base/rspfCommon.h>
#include <rspf/base/rspfKeywordlist.h>
#include <rspf/base/rspfKeywordNames.h>
#include <rspf/imaging/rspfImageSourceFactoryBase.h>
#include <rspf/imaging/rspfImageSourceFactoryRegistry.h>
#include <rspf/base/rspfRefPtr.h>
#include <rspf/base/rspfNumericProperty.h>
#include <rspf/base/rspfStringProperty.h>

#include "rspfOpenCVThresholdFilter.h"

RTTI_DEF1(rspfOpenCVThresholdFilter, "rspfOpenCVThresholdFilter", rspfImageSourceFilter)

rspfOpenCVThresholdFilter::rspfOpenCVThresholdFilter(rspfObject* owner)
:rspfImageSourceFilter(owner),
theTile(NULL),
theThreshold(10.0),
theMaxValue(255.0),
theThresholdType(1)
{
}

rspfOpenCVThresholdFilter::~rspfOpenCVThresholdFilter()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVThresholdFilter::getTile(const rspfIrect& tileRect, rspf_uint32 resLevel)
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


void rspfOpenCVThresholdFilter::initialize()
{
   if(theInputConnection)
   {
      rspfImageSourceFilter::initialize();
      theTile = new rspfU8ImageData(this,
				     theInputConnection->getNumberOfOutputBands(),   
                                     theInputConnection->getTileWidth(),
                                     theInputConnection->getTileHeight());  
      theTile->initialize();
   }
}

rspfScalarType rspfOpenCVThresholdFilter::getOutputScalarType() const
{
	if(!isSourceEnabled())
	{
		return rspfImageSourceFilter::getOutputScalarType();
	}

	return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVThresholdFilter::getNumberOfOutputBands() const
{
	if(!isSourceEnabled())
	{
		return rspfImageSourceFilter::getNumberOfOutputBands();
	}
   	return theInputConnection->getNumberOfOutputBands();
}

bool rspfOpenCVThresholdFilter::saveState(rspfKeywordlist& kwl, const char* prefix)const
{
	rspfImageSourceFilter::saveState(kwl, prefix);

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

void rspfOpenCVThresholdFilter::setThresholdtype(const rspfString& lookup)
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

bool rspfOpenCVThresholdFilter::loadState(const rspfKeywordlist& kwl, const char* prefix)
{
	rspfImageSourceFilter::loadState(kwl, prefix);

	const char* lookup = kwl.find(prefix, "threshold");
	if(lookup)
	{
		theThreshold = rspfString(lookup).toDouble();
 		printf("Read from spec file. threshold: %f\n",theThreshold);
	}
	lookup = kwl.find(prefix, "max_value");
	if(lookup)
	{
		theMaxValue = rspfString(lookup).toDouble();
 		printf("Read from spec file. max_value: %f\n",theMaxValue);
	}
	lookup = kwl.find(prefix, "threshold_type");
	printf("Read from spec file. threshold_type: %s\n",lookup);
	setThresholdtype(lookup);
  
	return true;
}

void rspfOpenCVThresholdFilter::runUcharTransformation(rspfImageData* tile)
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

rspfRefPtr<rspfProperty> rspfOpenCVThresholdFilter::getProperty(const rspfString& name)const
{
   if(name == "threshold")
   {
	   rspfProperty* prop = new rspfNumericProperty("threshold",
		   rspfString::toString(theThreshold));
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "max_value")
   {
	   rspfProperty* prop = new rspfNumericProperty("max_value",
		   rspfString::toString(theMaxValue));
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "threshold_type")
   {
		std::vector<rspfString> constraintList;
		getThresholdTypeList(constraintList);
		rspfString value = getThresholdTypeString();
		rspfProperty* prop = new rspfStringProperty("threshold_type",
														value,
														false,
														constraintList);
		prop->setCacheRefreshBit();
		return prop;
   }
   return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVThresholdFilter::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
        rspfImageSourceFilter::getPropertyNames(propertyNames);
        propertyNames.push_back("threshold");
        propertyNames.push_back("max_value");
        propertyNames.push_back("threshold_type");
}

void rspfOpenCVThresholdFilter::setProperty(rspfRefPtr<rspfProperty> property)
{
        if(!property.valid())
		{
			return;
		} 
        rspfString name = property->getName();

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
		  rspfImageSourceFilter::setProperty(property);
		}
}

void rspfOpenCVThresholdFilter::getThresholdTypeList(
   std::vector<rspfString>& list) const
{
   list.resize(7);

   list[0] = rspfString("CV_THRESH_BINARY");
   list[1] = rspfString("CV_THRESH_BINARY_INV");
   list[2] = rspfString("CV_THRESH_TRUNC");
   list[3] = rspfString("CV_THRESH_TOZERO");
   list[4] = rspfString("CV_THRESH_TOZERO_INV");
   //TODO: THRESH_MASK does not seem to exist any more in latest opencv
   list[5] = rspfString("CV_THRESH_MASK");
   list[6] = rspfString("CV_THRESH_OTSU");
}

rspfString rspfOpenCVThresholdFilter::getThresholdTypeString() const
{
   std::vector<rspfString> list;
   getThresholdTypeList(list);
   int type = theThresholdType>4?(theThresholdType-2):theThresholdType;
   printf("%d Threshold Type",type);
   return list[type];
}
