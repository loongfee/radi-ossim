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
#include "ossimOpenCVLogPolarFilter.h"
#include <ossim/imaging/ossimU8ImageData.h>
#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimCommon.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/imaging/ossimImageSourceFactoryBase.h>
#include <ossim/imaging/ossimImageSourceFactoryRegistry.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimNumericProperty.h>

RTTI_DEF1(ossimOpenCVLogPolarFilter, "ossimOpenCVLogPolarFilter", ossimImageSourceFilter)

ossimOpenCVLogPolarFilter::ossimOpenCVLogPolarFilter(ossimObject* owner)
   :ossimImageSourceFilter(owner),
    theTile(NULL),
    thecenter_x(100),
    thecenter_y(100),
    theM(40)
{
}

ossimOpenCVLogPolarFilter::~ossimOpenCVLogPolarFilter()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVLogPolarFilter::getTile(const ossimIrect& tileRect,
                                                                ossim_uint32 resLevel)
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getTile(tileRect,
                                             resLevel);
   }
   long w     = tileRect.width();
   long h     = tileRect.height();

   
   if(!theTile.valid()) initialize();
   if(!theTile.valid()) return 0;
   
   if(!theTile.valid()) return 0;
   
   ossimRefPtr<ossimImageData> data = 0;
   if(theInputConnection)
   {
      data  = theInputConnection->getTile(tileRect, resLevel);
   }
   else
   {
      return 0;
   }

   if(!data.valid()) return 0;
   if(data->getDataObjectStatus() == OSSIM_NULL ||
      data->getDataObjectStatus() == OSSIM_EMPTY)
   {
      return 0;
   }

   theTile->setImageRectangle(tileRect);
   theTile->makeBlank();
   
   theTile->setOrigin(tileRect.ul());
   runUcharTransformation(data.get());
   
   return theTile;
   
}

void ossimOpenCVLogPolarFilter::initialize()
{
   if(theInputConnection)
   {
      theTile = 0;
      
      theTile = new ossimU8ImageData(this,
                                     1,
                                     theInputConnection->getTileWidth(),
                                     theInputConnection->getTileHeight());  
      theTile->initialize();
   }
}

ossimScalarType ossimOpenCVLogPolarFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }
   
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVLogPolarFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return 1;
}

bool ossimOpenCVLogPolarFilter::saveState(ossimKeywordlist& kwl,
                                     const char* prefix)const
{
   ossimImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,
           "center_x",
           thecenter_x,
           true);
   kwl.add(prefix,
           "center_y",
           thecenter_y,
           true);
   kwl.add(prefix,
           "M",
           theM,
           true);
   
   return true;
}

bool ossimOpenCVLogPolarFilter::loadState(const ossimKeywordlist& kwl,
                                     const char* prefix)
{
   ossimImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "center_x");
   if(lookup)
   {
      thecenter_x = ossimString(lookup).toDouble();
   }
   lookup = kwl.find(prefix, "center_y");
   if(lookup)
   {
      thecenter_y = ossimString(lookup).toDouble();
   }
   lookup = kwl.find(prefix, "M");
   if(lookup)
   {
      theM = ossimString(lookup).toDouble();
   }
   return true;
}

void ossimOpenCVLogPolarFilter::runUcharTransformation(ossimImageData* tile)
{   
   IplImage *input;
   IplImage *output;

   input=cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
   output=cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
   char* bandSrc[3];//TODO: Use Vectors instead of arrays for variable band numbers
   char* bandDest;
   
      bandSrc[0]  = static_cast< char*>(tile->getBuf(0));

   input->imageData=bandSrc[0];
   bandDest = static_cast< char*>(theTile->getBuf());
   output->imageData=bandDest;

cvLogPolar( input, output, cvPoint2D32f(thecenter_x,thecenter_y),theM,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );


   theTile->validate();
}

void ossimOpenCVLogPolarFilter::setProperty(ossimRefPtr<ossimProperty> property)
{
	if(!property) return;
    ossimString name = property->getName();

    if(name == "center_x")
    {
            thecenter_x = property->valueToString().toDouble();
    }
	else if(name == "center_y")
    {
            thecenter_x = property->valueToString().toDouble();
    }
	else if(name == "M")
    {
            theM = property->valueToString().toDouble();
    }
	else
	{
	  ossimImageSourceFilter::setProperty(property);
	}
}

ossimRefPtr<ossimProperty> ossimOpenCVLogPolarFilter::getProperty(const ossimString& name)const
{
	if(name == "center_x")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(thecenter_x));
            numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "center_y")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(thecenter_y));
            numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "M")
	{
		ossimNumericProperty* numeric = new ossimNumericProperty(name,
        ossimString::toString(theM));
        numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
        numeric->setCacheRefreshBit();
        return numeric;
	}
	return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVLogPolarFilter::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
	ossimImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("center_x");
	propertyNames.push_back("center_y");
	propertyNames.push_back("M");
}