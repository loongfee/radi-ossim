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

#include "rspfOpenCVSmoothFilter.h"

#include <rspf/base/rspfRefPtr.h>

#include <rspf/base/rspfConstants.h>
#include <rspf/base/rspfCommon.h>
#include <rspf/base/rspfKeywordlist.h>
#include <rspf/base/rspfKeywordNames.h>
#include <rspf/imaging/rspfImageSourceFactoryBase.h>
#include <rspf/imaging/rspfImageSourceFactoryRegistry.h>
#include <rspf/imaging/rspfU8ImageData.h>
#include <rspf/base/rspfRefPtr.h>
#include <rspf/base/rspfNumericProperty.h>
#include <rspf/base/rspfStringProperty.h>

RTTI_DEF1(rspfOpenCVSmoothFilter, "rspfOpenCVSmoothFilter", rspfImageSourceFilter)


rspfOpenCVSmoothFilter::rspfOpenCVSmoothFilter(rspfObject* owner)
   :rspfImageSourceFilter(owner),
   theTile(NULL),
   theParam1(3),
   theParam2(3),
   theParam3(0),
   theParam4(0),
   theSmoothType(2)
{
}

rspfOpenCVSmoothFilter::~rspfOpenCVSmoothFilter()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVSmoothFilter::getTile(const rspfIrect& tileRect, rspf_uint32 resLevel) {

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

void rspfOpenCVSmoothFilter::initialize()
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

rspfScalarType rspfOpenCVSmoothFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }   
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVSmoothFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();

}

bool rspfOpenCVSmoothFilter::saveState(rspfKeywordlist& kwl, const char* prefix)const {
   
   rspfImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"param1",theParam1,true);
   kwl.add(prefix,"param2",theParam2,true);
   kwl.add(prefix,"param3",theParam3,true);
   kwl.add(prefix,"param4",theParam4,true);
   switch(theSmoothType) {
	case 0:
	   kwl.add(prefix,"smooth_type","CV_BLUR_NO_SCALE",true);
	break;
	case 1:
	   kwl.add(prefix,"smooth_type","CV_BLUR",true);
	break;
	case 2:
	   kwl.add(prefix,"smooth_type","CV_GAUSSIAN",true);
	break;
	case 3:
	   kwl.add(prefix,"smooth_type","CV_MEDIAN",true);
	break;
	case 4:
	   kwl.add(prefix,"smooth_type","CV_BILATERAL",true);
	break;
   }
   return true;
}

bool rspfOpenCVSmoothFilter::loadState(const rspfKeywordlist& kwl, const char* prefix) { 

   rspfImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "param1");
   if(lookup)
   {
      theParam1 = rspfString(lookup).toInt();
      printf("Read from spec file. param1: %d\n",theParam1);
   }
   lookup = kwl.find(prefix, "param2");
   if(lookup)
   {
      theParam2 = rspfString(lookup).toInt();
      printf("Read from spec file. param2: %d\n",theParam2);
   }
   lookup = kwl.find(prefix, "param3");
   if(lookup)
   {
      theParam3 = rspfString(lookup).toDouble();
      printf("Read from spec file. param3: %f\n",theParam3);
   }
   lookup = kwl.find(prefix, "param4");
   if(lookup)
   {
      theParam4 = rspfString(lookup).toDouble();
      printf("Read from spec file. param4: %f\n",theParam4);
   }
   lookup = kwl.find(prefix, "smooth_type");
   if(lookup)
   {
	   setSmoothType(lookup);
   }
   return true;
}

void rspfOpenCVSmoothFilter::runUcharTransformation(rspfImageData* tile)
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
		cvSmooth(input,output,theSmoothType,theParam1,theParam2,theParam3,theParam4);
		cvReleaseImageHeader(&input);
		cvReleaseImageHeader(&output);
	}

	theTile->validate(); 
 
   }

void rspfOpenCVSmoothFilter::setSmoothType(const rspfString lookup)
{
        if(strcmp(lookup,"CV_BLUR_NO_SCALE")==0){
		theSmoothType=0; 		
                printf("Read from spec file. smooth_type: %s\n",lookup.c_str());
	}
        else if(strcmp(lookup,"CV_BLUR")==0){
		theSmoothType=1; 		
                printf("Read from spec file. smooth_type: %s\n",lookup.c_str());
	}
        else if(strcmp(lookup,"CV_GAUSSIAN")==0){
		theSmoothType=2; 		
                printf("Read from spec file. smooth_type: %s\n",lookup.c_str());
	}
        else if(strcmp(lookup,"CV_MEDIAN")==0){
		theSmoothType=3; 		
                printf("Read from spec file. smooth_type: %s\n",lookup.c_str());
	}
        else if(strcmp(lookup,"CV_BILATERAL")==0){
		theSmoothType=4; 		
                printf("Read from spec file. smooth_type: %s\n",lookup.c_str());
	}
	else {
                printf("%s not supported as smooth_type parameter for OpenCVSmoothFilter!\nDefault smooth_type: CV_GAUSSIAN\n",lookup.c_str());
	}	
}

void rspfOpenCVSmoothFilter::getSmoothTypeList(
   std::vector<rspfString>& list) const
{
   list.resize(5);

   list[0] = rspfString("CV_BLUR_NO_SCALE");
   list[1] = rspfString("CV_BLUR");
   list[2] = rspfString("CV_GAUSSIAN");
   list[3] = rspfString("CV_MEDIAN");
   list[4] = rspfString("CV_BILATERAL");
}

rspfRefPtr<rspfProperty> rspfOpenCVSmoothFilter::getProperty(const rspfString& name)const
{
   if (name == "param1")
   {
	   rspfProperty* prop = new rspfNumericProperty("param1",
		   rspfString::toString(theParam1));
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "param2")
   {
	   rspfNumericProperty* prop = new rspfNumericProperty("param2",
		   rspfString::toString(theParam2));
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "param3")
   {
	   rspfNumericProperty* prop = new rspfNumericProperty("param3",
		   rspfString::toString(theParam3));
	   prop->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "param4")
   {
	   rspfNumericProperty* prop = new rspfNumericProperty("param4",
		   rspfString::toString(theParam4));
	   prop->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "smooth_type")
   {
		std::vector<rspfString> constraintList;
		getSmoothTypeList(constraintList);
		rspfString value = getSmoothTypeString();
		rspfProperty* prop = new rspfStringProperty("smooth_type",
														value,
														false,
														constraintList);
		prop->setCacheRefreshBit();
		return prop;
   }
   return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVSmoothFilter::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
    rspfImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("param1");
    propertyNames.push_back("param2");
	propertyNames.push_back("param3");
	propertyNames.push_back("param4");
    propertyNames.push_back("smooth_type");
}

void rspfOpenCVSmoothFilter::setProperty(rspfRefPtr<rspfProperty> property)
{
    if(!property.valid())
	{
		return;
	} 
    rspfString name = property->getName();
	if(name == "param1")
    {
		theParam1 = property->valueToString().toInt();
    }
    else if(name == "param2")
    {
		theParam2 = property->valueToString().toInt();
    }
    else if(name == "param3")
    {
        theParam3 = property->valueToString().toDouble();
    }
	else if(name == "param4")
    {
        theParam4 = property->valueToString().toDouble();
    }
	else if(name == "smooth_type")
	{
		setSmoothType(property->valueToString());
	}
	else
	{
		rspfImageSourceFilter::setProperty(property);
	}
}

rspfString rspfOpenCVSmoothFilter::getSmoothTypeString()const
{
   std::vector<rspfString> list;
   getSmoothTypeList(list);
   return list[theSmoothType];
}
