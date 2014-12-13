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
#include "ossimOpenCVErodeFilter.h"
#include <ossim/imaging/ossimU8ImageData.h>
#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimCommon.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/imaging/ossimImageSourceFactoryBase.h>
#include <ossim/imaging/ossimImageSourceFactoryRegistry.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimNumericProperty.h>

RTTI_DEF1(ossimOpenCVErodeFilter, "ossimOpenCVErodeFilter", ossimImageSourceFilter)

ossimOpenCVErodeFilter::ossimOpenCVErodeFilter(ossimObject* owner)
   :ossimImageSourceFilter(owner),
    theTile(NULL),
    theIterations(1) 
{
}

ossimOpenCVErodeFilter::~ossimOpenCVErodeFilter()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVErodeFilter::getTile(const ossimIrect& tileRect,
                                                                ossim_uint32 resLevel)
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

void ossimOpenCVErodeFilter::initialize()
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

ossimScalarType ossimOpenCVErodeFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }   
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVErodeFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();
}

bool ossimOpenCVErodeFilter::saveState(ossimKeywordlist& kwl,
                                     const char* prefix)const
{

   ossimImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"iterations",theIterations,true);
   
   return true;
  
}

bool ossimOpenCVErodeFilter::loadState(const ossimKeywordlist& kwl,
                                     const char* prefix)
{

   ossimImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "iterations");
   if(lookup)
   {
      theIterations = ossimString(lookup).toInt();
      printf("Read from spec file. iterations: %d\n",theIterations);
   }
   
   return true;
}

void ossimOpenCVErodeFilter::runUcharTransformation(ossimImageData* tile)
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
	cvErode(input,output,NULL,theIterations); // a 3x3 rectangular structuring element is used
	cvReleaseImageHeader(&input);
	cvReleaseImageHeader(&output);
	}
  theTile->validate(); 

}

/*
* Methods to expose thresholds for adjustment through the GUI
*/
void ossimOpenCVErodeFilter::setProperty(ossimRefPtr<ossimProperty> property)
{
        if(!property) return;
        ossimString name = property->getName();

        if(name == "iterations")
        {
                theIterations = property->valueToString().toInt();
        }
		else
		{
		  ossimImageSourceFilter::setProperty(property);
		}
}

ossimRefPtr<ossimProperty> ossimOpenCVErodeFilter::getProperty(const ossimString& name)const
{
        if(name == "iterations")
        {
                ossimNumericProperty* numeric = new ossimNumericProperty(name,
                        ossimString::toString(theIterations),
                        1, 5);
                numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_INT);
                numeric->setCacheRefreshBit();
                return numeric;
        }
        return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVErodeFilter::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
        ossimImageSourceFilter::getPropertyNames(propertyNames);
        propertyNames.push_back("iterations");
}