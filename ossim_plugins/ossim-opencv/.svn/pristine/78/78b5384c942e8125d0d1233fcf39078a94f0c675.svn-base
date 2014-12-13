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
#include "ossimOpenCVSobelFilter.h"
#include <ossim/imaging/ossimU8ImageData.h>
#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimCommon.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/imaging/ossimImageSourceFactoryBase.h>
#include <ossim/imaging/ossimImageSourceFactoryRegistry.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimNumericProperty.h>

RTTI_DEF1(ossimOpenCVSobelFilter, "ossimOpenCVSobelFilter", ossimImageSourceFilter)

ossimOpenCVSobelFilter::ossimOpenCVSobelFilter(ossimObject* owner)
   :ossimImageSourceFilter(owner),
    theTile(NULL),
    theXOrder(1),
    theYOrder(1),
    theApertureSize(3)
{
}

ossimOpenCVSobelFilter::ossimOpenCVSobelFilter(ossimImageSource* inputSource,
                                           int xorder,
                                           int yorder,
                                           int aperture_size)
   : ossimImageSourceFilter(NULL, inputSource),
     theTile(NULL),
     theXOrder(xorder),
     theYOrder(yorder),
     theApertureSize(aperture_size)
{
	
}

ossimOpenCVSobelFilter::ossimOpenCVSobelFilter(ossimObject* owner,
                                           ossimImageSource* inputSource,
                                           int xorder,
                                           int yorder,
                                           int aperture_size)
   : ossimImageSourceFilter(owner, inputSource),
     theTile(NULL),
     theXOrder(xorder),
     theYOrder(yorder),
     theApertureSize(aperture_size)
{
}

ossimOpenCVSobelFilter::~ossimOpenCVSobelFilter()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVSobelFilter::getTile(const ossimIrect& tileRect,
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

void ossimOpenCVSobelFilter::initialize()
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

ossimScalarType ossimOpenCVSobelFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }
   
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVSobelFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();
}

bool ossimOpenCVSobelFilter::saveState(ossimKeywordlist& kwl,  const char* prefix)const
{
   ossimImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"xorder",theXOrder,true);
   kwl.add(prefix,"yorder",theYOrder,true);
   kwl.add(prefix,"aperture_size",theApertureSize,true);
   
   return true;
}

bool ossimOpenCVSobelFilter::loadState(const ossimKeywordlist& kwl, const char* prefix)
{
   ossimImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "xorder");
   if(lookup)
   {
      theXOrder = ossimString(lookup).toInt();
      printf("Read from spec file. xorder: %d\n",theXOrder);
   }
   lookup = kwl.find(prefix, "yorder");
   if(lookup)
   {
      theYOrder = ossimString(lookup).toInt();
      printf("Read from spec file. yorder: %d\n",theYOrder);
   }
   lookup = kwl.find(prefix, "aperture_size");
   if(lookup)
   {
      setApertureSize(ossimString(lookup));
   }
   return true;
}

void ossimOpenCVSobelFilter::setApertureSize(const ossimString lookup)
{
	theApertureSize = lookup.toInt();
	if( theApertureSize < 1 || theApertureSize>7 || theApertureSize%2==0){
      printf("ERROR: aperture_size not supported! Must be 1, 3, 5 or 7! Default aperture_size: 3\n");
      theApertureSize=3;
	} else {
      printf("Read from spec file. aperture_size: %d\n",theApertureSize);
	}
}

void ossimOpenCVSobelFilter::runUcharTransformation(ossimImageData* tile) {
   
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
		IplImage * tmp = cvCreateImage(cvSize(tile->getWidth(),tile->getHeight()),IPL_DEPTH_16S,1);
		cvSobel(input,tmp,theXOrder,theYOrder,theApertureSize); 
		cvConvertScale(tmp,output);
		cvReleaseImageHeader(&input);
		cvReleaseImageHeader(&output);
		cvReleaseImage(&tmp);
	}

	theTile->validate(); 
}

void ossimOpenCVSobelFilter::setProperty(ossimRefPtr<ossimProperty> property)
{
	if(!property.valid())
	{
		return;
	} 
    ossimString name = property->getName();
	if(name == "xorder")
    {
		theXOrder = property->valueToString().toInt();
    }
    else if(name == "yorder")
    {
		theYOrder = property->valueToString().toInt();
    }
    else if(name == "aperture_size")
    {
        setApertureSize(property->valueToString());
    }
	else
	{
		ossimImageSourceFilter::setProperty(property);
	}
}

ossimRefPtr<ossimProperty> ossimOpenCVSobelFilter::getProperty(const ossimString& name)const
{
	if (name == "xorder")
   {
	   ossimProperty* prop = new ossimNumericProperty("xorder",
		   ossimString::toString(theXOrder));
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "yorder")
   {
	   ossimNumericProperty* prop = new ossimNumericProperty("yorder",
		   ossimString::toString(theYOrder));
	   prop->setCacheRefreshBit();
	   return prop;
   }
   else if (name == "aperture_size")
   {
	   ossimNumericProperty* prop = new ossimNumericProperty("aperture_size",
		   ossimString::toString(theApertureSize),1,7);
	   prop->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
	   prop->setCacheRefreshBit();
	   return prop;
   }
   return ossimImageSourceFilter::getProperty(name);
}
void ossimOpenCVSobelFilter::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
	ossimImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("xorder");
	propertyNames.push_back("yorder");
	propertyNames.push_back("aperture_size");
}
