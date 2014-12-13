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


#include "ossimOpenCVCannyFilter.h"

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

RTTI_DEF1(ossimOpenCVCannyFilter, "ossimOpenCVCannyFilter", ossimImageSourceFilter)

ossimOpenCVCannyFilter::ossimOpenCVCannyFilter(ossimObject* owner)
   :ossimImageSourceFilter(owner),
    theTile(NULL),
    theThreshold1(1.0/3.0),
    theThreshold2(1.0/3.0),
    theApertureSize(3)
{
}

ossimOpenCVCannyFilter::ossimOpenCVCannyFilter(ossimImageSource* inputSource,
                                           double threshold1 = 1.0/3.0, 
					   double threshold2 = 1.0/3.0, 
					   int apertureSize = 3)
   : ossimImageSourceFilter(NULL, inputSource),
     theTile(NULL),
     theThreshold1(threshold1),
     theThreshold2(threshold2),
     theApertureSize(apertureSize)
{
}

ossimOpenCVCannyFilter::ossimOpenCVCannyFilter(ossimObject* owner,
                                           ossimImageSource* inputSource,
                                           double threshold1 = 1.0/3.0, 
					   double threshold2 = 1.0/3.0, 
					   int apertureSize = 3)
   : ossimImageSourceFilter(owner, inputSource),
     theTile(NULL),
     theThreshold1(threshold1),
     theThreshold2(threshold2),
     theApertureSize(apertureSize)
{
}

ossimOpenCVCannyFilter::~ossimOpenCVCannyFilter()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVCannyFilter::getTile(const ossimIrect& tileRect, ossim_uint32 resLevel) 
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

void ossimOpenCVCannyFilter::initialize()
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

ossimScalarType ossimOpenCVCannyFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVCannyFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();
}

bool ossimOpenCVCannyFilter::saveState(ossimKeywordlist& kwl,
                                     const char* prefix)const
{
   ossimImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,
           "theshold1",
           theThreshold1,
           true);
   kwl.add(prefix,
           "threshold2",
           theThreshold2,
           true);
   kwl.add(prefix,
           "aperture_size",
           theApertureSize,
           true);
   
   return true;
}

bool ossimOpenCVCannyFilter::loadState(const ossimKeywordlist& kwl,
                                     const char* prefix)
{
   ossimImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "threshold1");
   if(lookup)
   {
      theThreshold1 = ossimString(lookup).toDouble();
      printf("Read from spec file. threshold1: %f\n",theThreshold1);
   }
   lookup = kwl.find(prefix, "threshold2");
   if(lookup)
   {
      theThreshold2 = ossimString(lookup).toDouble();
      printf("Read from spec file. threshold2: %f\n",theThreshold2);
   }
   lookup = kwl.find(prefix, "aperture_size");
   if(lookup)
   {
      theApertureSize = ossimString(lookup).toInt();
      printf("Read from spec file. aperture_size: %d\n",theApertureSize);
   }
   return true;
}

void ossimOpenCVCannyFilter::runUcharTransformation(ossimImageData* tile)
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
        	cvCanny(input, output, theThreshold1, theThreshold2, theApertureSize);
		cvReleaseImageHeader(&input);
		cvReleaseImageHeader(&output);
	}

	theTile->validate();   
}

void ossimOpenCVCannyFilter::setProperty(ossimRefPtr<ossimProperty> property)
{
	if(!property) return;
    ossimString name = property->getName();

    if(name == "theshold1")
    {
            theThreshold1 = property->valueToString().toDouble();
    }
	else if(name == "theshold2")
    {
            theThreshold2 = property->valueToString().toDouble();
    }
	else if(name == "aperture_size")
    {
            theApertureSize = property->valueToString().toInt();
    }
	else
	{
	  ossimImageSourceFilter::setProperty(property);
	}
}

ossimRefPtr<ossimProperty> ossimOpenCVCannyFilter::getProperty(const ossimString& name)const
{
	if(name == "threshold1")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theThreshold1));
            numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	if(name == "threshold2")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theThreshold2));
            numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	if(name == "aperture_size")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theApertureSize),3,7);
			//TODO: The aperture size can only be 3,5,7.
            numeric->setCacheRefreshBit();
            return numeric;
    }
    return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVCannyFilter::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
	ossimImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("threshold1");
	propertyNames.push_back("threshold2");
	propertyNames.push_back("aperture_size");
}