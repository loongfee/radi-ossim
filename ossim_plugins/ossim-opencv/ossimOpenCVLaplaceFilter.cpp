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

#include "ossimOpenCVLaplaceFilter.h"

RTTI_DEF1(ossimOpenCVLaplaceFilter, "ossimOpenCVLaplaceFilter", ossimImageSourceFilter)

ossimOpenCVLaplaceFilter::ossimOpenCVLaplaceFilter(ossimObject* owner)
   :ossimImageSourceFilter(owner),
   theApertureSize(3)
{
}

ossimOpenCVLaplaceFilter::ossimOpenCVLaplaceFilter(ossimImageSource* inputSource, int aperture_size)
   : ossimImageSourceFilter(NULL, inputSource),
     theTile(NULL),
     theApertureSize(aperture_size)
{
}

ossimOpenCVLaplaceFilter::ossimOpenCVLaplaceFilter(ossimObject* owner,
                                           ossimImageSource* inputSource,
                                           int aperture_size)
   : ossimImageSourceFilter(owner, inputSource),
     theTile(NULL),
     theApertureSize(aperture_size)
{
}

ossimOpenCVLaplaceFilter::~ossimOpenCVLaplaceFilter()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVLaplaceFilter::getTile(const ossimIrect& tileRect,
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

void ossimOpenCVLaplaceFilter::initialize()
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

ossimScalarType ossimOpenCVLaplaceFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }
   
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVLaplaceFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();
}

bool ossimOpenCVLaplaceFilter::saveState(ossimKeywordlist& kwl,  const char* prefix)const
{
   ossimImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"aperture_size",theApertureSize,true);
   
   return true;
}

bool ossimOpenCVLaplaceFilter::loadState(const ossimKeywordlist& kwl, const char* prefix)
{
   ossimImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "aperture_size");
   if(lookup)
   {
      setApertureSize(ossimString(lookup).toInt());
   }
   return true;
}

void ossimOpenCVLaplaceFilter::runUcharTransformation(ossimImageData* tile) {
   
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

void ossimOpenCVLaplaceFilter::setApertureSize(const int apsize)
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
void ossimOpenCVLaplaceFilter::setProperty(ossimRefPtr<ossimProperty> property)
{
        if(!property) return;
        ossimString name = property->getName();

        if(name == "aperture_size")
        {
                setApertureSize(property->valueToString().toInt());
        }
		else
		{
		  ossimImageSourceFilter::setProperty(property);
		}
}

ossimRefPtr<ossimProperty> ossimOpenCVLaplaceFilter::getProperty(const ossimString& name)const
{
        if(name == "aperture_size")
        {
                ossimNumericProperty* numeric = new ossimNumericProperty(name,
                        ossimString::toString(theApertureSize),
                        1, 7);
                numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_INT);
                numeric->setCacheRefreshBit();
                return numeric;
        }
        return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVLaplaceFilter::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
        ossimImageSourceFilter::getPropertyNames(propertyNames);
        propertyNames.push_back("aperture_size");
}