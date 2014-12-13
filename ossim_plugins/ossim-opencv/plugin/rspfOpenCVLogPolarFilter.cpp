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
#include "rspfOpenCVLogPolarFilter.h"
#include <rspf/imaging/rspfU8ImageData.h>
#include <rspf/base/rspfConstants.h>
#include <rspf/base/rspfCommon.h>
#include <rspf/base/rspfKeywordlist.h>
#include <rspf/base/rspfKeywordNames.h>
#include <rspf/imaging/rspfImageSourceFactoryBase.h>
#include <rspf/imaging/rspfImageSourceFactoryRegistry.h>
#include <rspf/base/rspfRefPtr.h>
#include <rspf/base/rspfNumericProperty.h>

RTTI_DEF1(rspfOpenCVLogPolarFilter, "rspfOpenCVLogPolarFilter", rspfImageSourceFilter)

rspfOpenCVLogPolarFilter::rspfOpenCVLogPolarFilter(rspfObject* owner)
   :rspfImageSourceFilter(owner),
    theTile(NULL),
    thecenter_x(100),
    thecenter_y(100),
    theM(40)
{
}

rspfOpenCVLogPolarFilter::~rspfOpenCVLogPolarFilter()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVLogPolarFilter::getTile(const rspfIrect& tileRect,
                                                                rspf_uint32 resLevel)
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getTile(tileRect,
                                             resLevel);
   }
   long w     = tileRect.width();
   long h     = tileRect.height();

   
   if(!theTile.valid()) initialize();
   if(!theTile.valid()) return 0;
   
   if(!theTile.valid()) return 0;
   
   rspfRefPtr<rspfImageData> data = 0;
   if(theInputConnection)
   {
      data  = theInputConnection->getTile(tileRect, resLevel);
   }
   else
   {
      return 0;
   }

   if(!data.valid()) return 0;
   if(data->getDataObjectStatus() == RSPF_NULL ||
      data->getDataObjectStatus() == RSPF_EMPTY)
   {
      return 0;
   }

   theTile->setImageRectangle(tileRect);
   theTile->makeBlank();
   
   theTile->setOrigin(tileRect.ul());
   runUcharTransformation(data.get());
   
   return theTile;
   
}

void rspfOpenCVLogPolarFilter::initialize()
{
   if(theInputConnection)
   {
      theTile = 0;
      
      theTile = new rspfU8ImageData(this,
                                     1,
                                     theInputConnection->getTileWidth(),
                                     theInputConnection->getTileHeight());  
      theTile->initialize();
   }
}

rspfScalarType rspfOpenCVLogPolarFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }
   
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVLogPolarFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return 1;
}

bool rspfOpenCVLogPolarFilter::saveState(rspfKeywordlist& kwl,
                                     const char* prefix)const
{
   rspfImageSourceFilter::saveState(kwl, prefix);

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

bool rspfOpenCVLogPolarFilter::loadState(const rspfKeywordlist& kwl,
                                     const char* prefix)
{
   rspfImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "center_x");
   if(lookup)
   {
      thecenter_x = rspfString(lookup).toDouble();
   }
   lookup = kwl.find(prefix, "center_y");
   if(lookup)
   {
      thecenter_y = rspfString(lookup).toDouble();
   }
   lookup = kwl.find(prefix, "M");
   if(lookup)
   {
      theM = rspfString(lookup).toDouble();
   }
   return true;
}

void rspfOpenCVLogPolarFilter::runUcharTransformation(rspfImageData* tile)
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

void rspfOpenCVLogPolarFilter::setProperty(rspfRefPtr<rspfProperty> property)
{
	if(!property) return;
    rspfString name = property->getName();

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
	  rspfImageSourceFilter::setProperty(property);
	}
}

rspfRefPtr<rspfProperty> rspfOpenCVLogPolarFilter::getProperty(const rspfString& name)const
{
	if(name == "center_x")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(thecenter_x));
            numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "center_y")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(thecenter_y));
            numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
    }
	else if(name == "M")
	{
		rspfNumericProperty* numeric = new rspfNumericProperty(name,
        rspfString::toString(theM));
        numeric->setNumericType(rspfNumericProperty::rspfNumericPropertyType_FLOAT64);
        numeric->setCacheRefreshBit();
        return numeric;
	}
	return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVLogPolarFilter::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
	rspfImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("center_x");
	propertyNames.push_back("center_y");
	propertyNames.push_back("M");
}