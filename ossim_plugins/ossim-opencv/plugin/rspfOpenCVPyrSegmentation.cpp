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
#include "rspfOpenCVPyrSegmentation.h"
#include <rspf/imaging/rspfU8ImageData.h>
#include <rspf/base/rspfConstants.h>
#include <rspf/base/rspfCommon.h>
#include <rspf/base/rspfKeywordlist.h>
#include <rspf/base/rspfKeywordNames.h>
#include <rspf/imaging/rspfImageSourceFactoryBase.h>
#include <rspf/imaging/rspfImageSourceFactoryRegistry.h>
#include <rspf/base/rspfRefPtr.h>
#include <rspf/imaging/rspfAnnotationPolyObject.h>
#include <rspf/base/rspfNumericProperty.h>

#include <opencv2/legacy/legacy.hpp>

RTTI_DEF1(rspfOpenCVPyrSegmentation, "rspfOpenCVPyrSegmentation", rspfImageSourceFilter)

rspfOpenCVPyrSegmentation::rspfOpenCVPyrSegmentation(rspfObject* owner)
   :rspfImageSourceFilter(owner),
    theTile(NULL),
    theLevel(4),
    theThreshold1(255.0),
    theThreshold2(30.0)	
{
}

rspfOpenCVPyrSegmentation::~rspfOpenCVPyrSegmentation()
{
}

rspfRefPtr<rspfImageData> rspfOpenCVPyrSegmentation::getTile(const rspfIrect& tileRect, rspf_uint32 resLevel)
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


void rspfOpenCVPyrSegmentation::initialize()
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

rspfScalarType rspfOpenCVPyrSegmentation::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getOutputScalarType();
   }
   
   return RSPF_UCHAR;
}

rspf_uint32 rspfOpenCVPyrSegmentation::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return rspfImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();
}

bool rspfOpenCVPyrSegmentation::saveState(rspfKeywordlist& kwl, const char* prefix)const
{
   rspfImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,"level",theLevel,true);
   kwl.add(prefix,"threshold1",theThreshold1,true);
   kwl.add(prefix,"threshold2",theThreshold2,true); 

   return true;
}

bool rspfOpenCVPyrSegmentation::loadState(const rspfKeywordlist& kwl,
                                     const char* prefix)
{
   rspfImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "level");
   if(lookup)
   {
      theLevel = rspfString(lookup).toInt();
      printf("Read from spec file. level: %d\n",theLevel);
   }
   lookup = kwl.find(prefix, "threshold1");
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

   return true;
}

void rspfOpenCVPyrSegmentation::runUcharTransformation(rspfImageData* tile)
{   

	IplImage *input;
	IplImage *output;

	char* bSrc;
	char* bDst;

	CvSeq *comp; // pointer to the output sequence of the segmented components
	//CvConnectedComp * cc; // pointer to a segmented component
	int n_comp;  // number of segmented components in the output sequence
	CvMemStorage *storage; 
	int nChannels = tile->getNumberOfBands();

	for(int k=0; k<nChannels; k++) {

		printf("Channel %d\n",k);

		// Pyramids segmentation
		input=cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
		output=cvCreateImageHeader(cvSize(tile->getWidth(),tile->getHeight()),8,1);
		bSrc = static_cast<char*>(tile->getBuf(k));
		input->imageData=bSrc;
		bDst = static_cast<char*>(theTile->getBuf(k));
		output->imageData=bDst;
		storage = cvCreateMemStorage (0); // creates a 64K memory storage block 
		cvPyrSegmentation(input, output, storage, &comp, theLevel, theThreshold1, theThreshold2);
		n_comp=comp->total;

		/* FIXME cvPyrSegmentation does not fill 'contour' in CvConnectedComponent struct...
		printf("Number of segments found: %d\n",n_comp);
		int i=0;
		while (i<n_comp)
		{
			printf("Segment %d\n",i);
			cc=(CvConnectedComp*)cvGetSeqElem(comp,i);
			printf("Area: %f\n",cc->area);
			CvScalar s = (CvScalar)cc->value; 
			printf("Value: (%f, %f, %f, %f)\n",s.val[0],s.val[1],s.val[2],s.val[3]);
			printf("Contour: %p\n",cc->contour);
			//for (int j=0; j<currentComp->contour->total;j++)
			//{
			//	printf("(%d,%d) ",((CvPoint*)cvGetSeqElem(currentComp->contour,j))->x,((CvPoint*)cvGetSeqElem(currentComp->contour,j))->y);
			//}
			i++;
		} 
		*/ 
		cvReleaseImageHeader(&input);
		cvReleaseImageHeader(&output);
		cvReleaseMemStorage(&storage);
	}	
  
	theTile->validate(); 



}

void rspfOpenCVPyrSegmentation::setProperty(rspfRefPtr<rspfProperty> property)
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
	else if(name == "level")
    {
            theLevel = property->valueToString().toInt();
    }
	else
	{
	  rspfImageSourceFilter::setProperty(property);
	}
}

rspfRefPtr<rspfProperty> rspfOpenCVPyrSegmentation::getProperty(const rspfString& name)const
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
	if(name == "level")
    {
            rspfNumericProperty* numeric = new rspfNumericProperty(name,
                    rspfString::toString(theLevel),1,6);
            numeric->setCacheRefreshBit();
            return numeric;
    }
    return rspfImageSourceFilter::getProperty(name);
}

void rspfOpenCVPyrSegmentation::getPropertyNames(std::vector<rspfString>& propertyNames)const
{
	rspfImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("threshold1");
	propertyNames.push_back("threshold2");
	propertyNames.push_back("level");
}