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

#include "rspfOpenCVLaplaceFilter.h"

RTTI_DEF1(rspfOpenCVLaplaceFilter, "rspfOpenCVLaplaceFilter", rspfImageSourceFilter)

rspfOpenCVLaplaceFilter::rspfOpenCVLaplaceFilter(rspfObject* owner)
   :rspfImageSourceFilter(owner),
   theApertureSize(3)
{
}

rspfOpenCVLaplaceFilter::rspfOpenCVLaplaceFilter(rspfImageSource* inputSource, int aperture_size)
   : rspfImageSourceFilter(NULL, inputSource),
     theTile(NULL),
     theApertureSize(aperture_size)
{
}

rspfOpenCVLaplaceFilter::rspfOpenCVLaplaceFilter(rspfObject* owner,
                                           rspfImageSource* inputSource,
                                           int aperture_size)
   : rspfImageSourceFilter(owner, inputSource),
     theTile(NULL),
     theApertureSize(aperture_size)
{
}

rspfOpenCVLaplaceFilter::~rspfOpenCVLaplaceFilter()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVLaplaceFilter::getTile(const rspfIrect& tileRect,
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

void rspfOpenCVLaplaceFilter::initialize()
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

rspfScalarType rspfOpenCVLaplaceFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }
   
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVLaplaceFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();
}

bool rspfOpenCVLaplaceFilter::saveState(rspfKeywordlist& kwl,  const char* prefix)const
{
   rspfImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"aperture_size",theApertureSize,true);
   
   return true;
}

bool rspfOpenCVLaplaceFilter::loadState(const rspfKeywordlist& kwl, const char* prefix)
{
   rspfImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "aperture_size");
   if(lookup)
   {
      setApertureSize(rspfString(lookup).toInt());
   }
   return true;
}

void rspfOpenCVLaplaceFilter::runUcharTransformation(rspfImageData* tile) {
   
	IplImage *input;
	IplImage *output;

	char* bSrc;
	char* bDst;
	
	int nChannels = tile->getNumberOfBands();

	for(int k=0; k<nChannels; k++) {
		//printf("Channel %d\n",k);
		input=cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
		output=cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
		bSrc = static_cast<char*>(tile->getBuf(k));
		input->imageData=bSrc;
		bDst = static_cast<char*>(theTile->getBuf(k));
		output->imageData=bDst;
		IplImage * tmp = cvCreateImage(cvSize(tile->getWidth(),tile->getHeight()),IPL_DEPTH_16S,1);
		cvLaplace(input,tmp,theApertureSize); 
		cvConvertScale(tmp,output);
		cvReleaseImageHeader(&input);
		cvReleaseImageHeader(&output);
		cvReleaseImage(&tmp);
	}

	theTile->validate(); 
}

void rspfOpenCVLaplaceFilter::setApertureSize(const int apsize)
{
  theApertureSize = apsize;
  if( theApertureSize < 1 || theApertureSize>7 || theApertureSize%2==0){
      printf("ERROR: aperture_size not supported! Must be 1, 3, 5 or 7! Default aperture_size: 3\n");
      theApertureSize=3;
  } else {
      printf("Read from spec file. aperture_size: %d\n",theApertureSize);
  }
}

/*
* Methods to expose thresholds for adjustment through the GUI
*/
void rspfOpenCVLaplaceFilter::setProperty(rspfRefPtr<rspfProperty> property)
{
        if(!property) return;
        rspfString name = property->getName();

        if(name == "aperture_size")
        {
                setApertureSize(property->valueToString().toInt());
        }
		else
		{
		  rspfImageSourceFilter::setProperty(property);
		}
}

rspfRefPtr<rspfProperty> rspfOpenCVLaplaceFilter::getProperty(const rspfString& name)const
{
        if(name == "aperture_size")
        {
                rspfNumericProperty* numeric = new rspfNumericProperty(name,
                        rspfString::toString(theApertureSize),
                        1, 7);
                numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_INT);
                numeric->setCacheRefreshBit();
                return numeric;
        }
        return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVLaplaceFilter::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
        rspfImageSourceFilter::getPropertyNames(propertyNames);
        propertyNames.push_back("aperture_size");
}