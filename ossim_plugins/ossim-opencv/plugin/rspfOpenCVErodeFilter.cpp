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
#include "rspfOpenCVErodeFilter.h"
#include <rspf/imaging/rspfU8ImageData.h>
#include <rspf/base/rspfConstants.h>
#include <rspf/base/rspfCommon.h>
#include <rspf/base/rspfKeywordlist.h>
#include <rspf/base/rspfKeywordNames.h>
#include <rspf/imaging/rspfImageSourceFactoryBase.h>
#include <rspf/imaging/rspfImageSourceFactoryRegistry.h>
#include <rspf/base/rspfRefPtr.h>
#include <rspf/base/rspfNumericProperty.h>

RTTI_DEF1(rspfOpenCVErodeFilter, "rspfOpenCVErodeFilter", rspfImageSourceFilter)

rspfOpenCVErodeFilter::rspfOpenCVErodeFilter(rspfObject* owner)
   :rspfImageSourceFilter(owner),
    theTile(NULL),
    theIterations(1) 
{
}

rspfOpenCVErodeFilter::~rspfOpenCVErodeFilter()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVErodeFilter::getTile(const rspfIrect& tileRect,
                                                                rspf_uint32 resLevel)
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

void rspfOpenCVErodeFilter::initialize()
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

rspfScalarType rspfOpenCVErodeFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }   
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVErodeFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();
}

bool rspfOpenCVErodeFilter::saveState(rspfKeywordlist& kwl,
                                     const char* prefix)const
{

   rspfImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"iterations",theIterations,true);
   
   return true;
  
}

bool rspfOpenCVErodeFilter::loadState(const rspfKeywordlist& kwl,
                                     const char* prefix)
{

   rspfImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "iterations");
   if(lookup)
   {
      theIterations = rspfString(lookup).toInt();
      printf("Read from spec file. iterations: %d\n",theIterations);
   }
   
   return true;
}

void rspfOpenCVErodeFilter::runUcharTransformation(rspfImageData* tile)
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
void rspfOpenCVErodeFilter::setProperty(rspfRefPtr<rspfProperty> property)
{
        if(!property) return;
        rspfString name = property->getName();

        if(name == "iterations")
        {
                theIterations = property->valueToString().toInt();
        }
		else
		{
		  rspfImageSourceFilter::setProperty(property);
		}
}

rspfRefPtr<rspfProperty> rspfOpenCVErodeFilter::getProperty(const rspfString& name)const
{
        if(name == "iterations")
        {
                rspfNumericProperty* numeric = new rspfNumericProperty(name,
                        rspfString::toString(theIterations),
                        1, 5);
                numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_INT);
                numeric->setCacheRefreshBit();
                return numeric;
        }
        return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVErodeFilter::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
        rspfImageSourceFilter::getPropertyNames(propertyNames);
        propertyNames.push_back("iterations");
}