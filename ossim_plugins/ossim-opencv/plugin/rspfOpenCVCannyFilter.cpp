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


#include "rspfOpenCVCannyFilter.h"

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

RTTI_DEF1(rspfOpenCVCannyFilter, "rspfOpenCVCannyFilter", rspfImageSourceFilter)

rspfOpenCVCannyFilter::rspfOpenCVCannyFilter(rspfObject* owner)
   :rspfImageSourceFilter(owner),
    theTile(NULL),
    theThreshold1(1.0/3.0),
    theThreshold2(1.0/3.0),
    theApertureSize(3)
{
}

rspfOpenCVCannyFilter::rspfOpenCVCannyFilter(rspfImageSource* inputSource,
                                           double threshold1 = 1.0/3.0, 
					   double threshold2 = 1.0/3.0, 
					   int apertureSize = 3)
   : rspfImageSourceFilter(NULL, inputSource),
     theTile(NULL),
     theThreshold1(threshold1),
     theThreshold2(threshold2),
     theApertureSize(apertureSize)
{
}

rspfOpenCVCannyFilter::rspfOpenCVCannyFilter(rspfObject* owner,
                                           rspfImageSource* inputSource,
                                           double threshold1 = 1.0/3.0, 
					   double threshold2 = 1.0/3.0, 
					   int apertureSize = 3)
   : rspfImageSourceFilter(owner, inputSource),
     theTile(NULL),
     theThreshold1(threshold1),
     theThreshold2(threshold2),
     theApertureSize(apertureSize)
{
}

rspfOpenCVCannyFilter::~rspfOpenCVCannyFilter()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVCannyFilter::getTile(const rspfIrect& tileRect, rspf_uint32 resLevel) 
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

void rspfOpenCVCannyFilter::initialize()
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

rspfScalarType rspfOpenCVCannyFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVCannyFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();
}

bool rspfOpenCVCannyFilter::saveState(rspfKeywordlist& kwl,
                                     const char* prefix)const
{
   rspfImageSourceFilter::saveState(kwl, prefix);

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

bool rspfOpenCVCannyFilter::loadState(const rspfKeywordlist& kwl,
                                     const char* prefix)
{
   rspfImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "threshold1");
   if(lookup)
   {
      theThreshold1 = rspfString(lookup).toDouble();
      printf("Read from spec file. threshold1: %f\n",theThreshold1);
   }
   lookup = kwl.find(prefix, "threshold2");
   if(lookup)
   {
      theThreshold2 = rspfString(lookup).toDouble();
      printf("Read from spec file. threshold2: %f\n",theThreshold2);
   }
   lookup = kwl.find(prefix, "aperture_size");
   if(lookup)
   {
      theApertureSize = rspfString(lookup).toInt();
      printf("Read from spec file. aperture_size: %d\n",theApertureSize);
   }
   return true;
}

void rspfOpenCVCannyFilter::runUcharTransformation(rspfImageData* tile)
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

void rspfOpenCVCannyFilter::setProperty(rspfRefPtr<rspfProperty> property)
{
	if(!property) return;
    rspfString name = property->getName();

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
	  rspfImageSourceFilter::setProperty(property);
	}
}

rspfRefPtr<rspfProperty> rspfOpenCVCannyFilter::getProperty(const rspfString& name)const
{
	if(name == "threshold1")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theThreshold1));
            numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	if(name == "threshold2")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theThreshold2));
            numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	if(name == "aperture_size")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theApertureSize),3,7);
			//TODO: The aperture size can only be 3,5,7.
            numeric->setCacheRefreshBit();
            return numeric;
    }
    return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVCannyFilter::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
	rspfImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("threshold1");
	propertyNames.push_back("threshold2");
	propertyNames.push_back("aperture_size");
}