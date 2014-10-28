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


#include "ossimOpenCVLsdFilter.h"

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

#include "lsd.h"

RTTI_DEF1(ossimOpenCVLsdFilter, "ossimOpenCVLsdFilter", ossimImageSourceFilter)

ossimOpenCVLsdFilter::ossimOpenCVLsdFilter(ossimObject* owner)
   :ossimImageSourceFilter(owner),
	m_nCount(0),
    theTile(NULL),
    theScale(0.8),
	theOutASCII("")
{
	FILE *pf = fopen(theOutASCII, "w+");
	if(pf)	fclose(pf);	
}

ossimOpenCVLsdFilter::ossimOpenCVLsdFilter(ossimImageSource* inputSource,
                                           double scale = 0.8, ossimFilename outASCII = "", int saveImage = 0)
   : ossimImageSourceFilter(NULL, inputSource),
	m_nCount(0),
     theTile(NULL),
	 theScale(scale),
	 theOutASCII(outASCII),
	 theSaveImage(saveImage)
{
	FILE *pf = fopen(theOutASCII, "w+");
	if(pf)	fclose(pf);	
}

ossimOpenCVLsdFilter::ossimOpenCVLsdFilter(ossimObject* owner,
                                           ossimImageSource* inputSource,
                                           double scale = 0.8, ossimFilename outASCII = "", int saveImage = 0)
   : ossimImageSourceFilter(owner, inputSource),
	m_nCount(0),
     theTile(NULL),
	 theScale(scale),
	 theOutASCII(outASCII),
	 theSaveImage(saveImage)
{
}

ossimOpenCVLsdFilter::~ossimOpenCVLsdFilter()
{
}

ossimRefPtr<ossimImageData> ossimOpenCVLsdFilter::getTile(const ossimIrect& tileRect, ossim_uint32 resLevel) 
{

	if(!isSourceEnabled())
   	{
	      return ossimImageSourceFilter::getTile(tileRect, resLevel);
	}
	long w     = tileRect.width();
	long h     = tileRect.height();
   
	if (1 == theSaveImage)
	{
		if(!theTile.valid()) initialize();
		if(!theTile.valid()) return 0;
	}	
  
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

	if (1 == theSaveImage)
	{
		theTile->setImageRectangle(tileRect);
		theTile->makeBlank();

		theTile->setOrigin(tileRect.ul());
	}
	runUcharTransformation(data.get(), tileRect.ul());
   
	//printf("Tile (%d,%d) finished!\n",tileRect.ul().x,tileRect.ul().y); 	
   	return theTile;
  
   
}

void ossimOpenCVLsdFilter::initialize()
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

ossimScalarType ossimOpenCVLsdFilter::getOutputScalarType() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getOutputScalarType();
   }
   return OSSIM_UCHAR;
}

ossim_uint32 ossimOpenCVLsdFilter::getNumberOfOutputBands() const
{
   if(!isSourceEnabled())
   {
      return ossimImageSourceFilter::getNumberOfOutputBands();
   }
   return theInputConnection->getNumberOfOutputBands();
}

bool ossimOpenCVLsdFilter::saveState(ossimKeywordlist& kwl,
                                     const char* prefix)const
{
   ossimImageSourceFilter::saveState(kwl, prefix);

   kwl.add(prefix,
           "scale",
           theScale,
		   true);
   kwl.add(prefix,
	   "outASCII",
	   theOutASCII,
	   true);
   kwl.add(prefix,
	   "saveImage",
	   theSaveImage,
	   true);
   kwl.add(prefix,
	   "lineSegments",
	   theLineSegments,
	   true);
   
   return true;
}

bool ossimOpenCVLsdFilter::loadState(const ossimKeywordlist& kwl,
                                     const char* prefix)
{
   ossimImageSourceFilter::loadState(kwl, prefix);

   const char* lookup = kwl.find(prefix, "scale");
   if(lookup)
   {
      theScale = ossimString(lookup).toDouble();
      printf("Read from spec file. scale: %f\n",theScale);
   }
   lookup = kwl.find(prefix, "outASCII");
   if(lookup)
   {
	   theOutASCII = ossimString(lookup);
	   printf("Read from spec file. outASCII: %s\n", theOutASCII);
   }
   lookup = kwl.find(prefix, "saveImage");
   if(lookup)
   {
	   theSaveImage = ossimString(lookup).toInt();
	   printf("Read from spec file. saveImage: %d\n", theSaveImage);
   }
   lookup = kwl.find(prefix, "lineSegments");
   if(lookup)
   {
	   theLineSegments = ossimString(lookup).toInt();
	   printf("Read from spec file. lineSegments: %d\n", theLineSegments);
   }
   return true;
}

void ossimOpenCVLsdFilter::runUcharTransformation(ossimImageData* tile, ossimIpt offset)
{   

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
        	cvLsd(input, output, offset, theScale);
		cvReleaseImageHeader(&input);
		cvReleaseImageHeader(&output);
	}

	theTile->validate();   
}

void ossimOpenCVLsdFilter::setProperty(ossimRefPtr<ossimProperty> property)
{
	if(!property) return;
    ossimString name = property->getName();

    if(name == "scale")
    {
            theScale = property->valueToString().toDouble();
	}
	else if(name == "outASCII")
	{
		theOutASCII = property->valueToString();
	}
	else if(name == "saveImage")
	{
		theSaveImage = property->valueToString().toInt();
	}
	else
	{
	  ossimImageSourceFilter::setProperty(property);
	}
}

ossimRefPtr<ossimProperty> ossimOpenCVLsdFilter::getProperty(const ossimString& name)const
{
	if(name == "scale")
    {
            ossimNumericProperty* numeric = new ossimNumericProperty(name,
                    ossimString::toString(theScale));
            numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_FLOAT64);
            numeric->setCacheRefreshBit();
            return numeric;
	}
	if(name == "outASCII")
	{
		ossimNumericProperty* numeric = new ossimNumericProperty(name,
			ossimString::toString(theOutASCII));
		return numeric;
	}
	if(name == "saveImage")
	{
		ossimNumericProperty* numeric = new ossimNumericProperty(name,
			ossimString::toString(theSaveImage));
		numeric->setNumericType(ossimNumericProperty::ossimNumericPropertyType_INT);
		numeric->setCacheRefreshBit();
		return numeric;
	}
	if(name == "lineSegments")
	{
		ossimNumericProperty* numeric = new ossimNumericProperty(name, theLineSegments);
		return numeric;
	}
    return ossimImageSourceFilter::getProperty(name);
}

void ossimOpenCVLsdFilter::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
	ossimImageSourceFilter::getPropertyNames(propertyNames);
	propertyNames.push_back("scale");
	propertyNames.push_back("outASCII");
	propertyNames.push_back("saveImage");
	propertyNames.push_back("lineSegments");
}

bool ossimOpenCVLsdFilter::cvLsd(IplImage* input, IplImage* output, ossimIpt offset, double scale/* = 0.8*/)
{
	double * image;
	double * out;

	int X = input->width;
	int Y = input->height;


	/* create a simple image: left half black, right half gray */
	image = (double *) malloc( X * Y * sizeof(double) );
	if( image == NULL )
	{
		fprintf(stderr,"error: not enough memory\n");
		exit(EXIT_FAILURE);
	}

	for(int i=0;i</*grey*/input->width;i++)
	{
		for(int j=0;j</*grey*/input->height;j++)
		{
			CvScalar s= cvGet2D(/*grey*/input,j,i);
			double pix= s.val[0];
			image[ i + j * X ]= pix;
		}
	}

	/* call LSD */
	int n;
	//out = lsd(&n, image, X, Y);
	out = lsd_scale(&n, image, X, Y, scale);

	//for (int i = 0;i < n;i++)
	//{
	//	//double r[7];
	//	//memcpy(r, out+i*7, n*sizeof(double));
	//	//theLinesegments.push_back(r);
	//	for (int j = 0;j < 7;j++)
	//	{
	//		ossimString str = ossimString::toString(out[i*7+j]);
	//		theLineSegments += str + "";
	//	}
	//	
	//}
	
	FILE* pf;
	if (0 == m_nCount++)
	{
		pf = fopen(theOutASCII.c_str(), "w+");
	}
	else
	{
		pf = fopen(theOutASCII.c_str(), "a+");
	}
	if (pf)
	{
		for (int i = 0;i < n;i++)
		{
			//x1,y1,x2,y2,width,p,-log10(NFA)
			fprintf(pf, "%15lf%15lf%15lf%15lf%15lf%15lf%15lf\n",
				out[ i * 7 + 0]+offset.x, out[ i * 7 + 1]+offset.y, out[ i * 7 + 2]+offset.x, out[ i * 7 + 3]+offset.y, out[ i * 7 + 4], out[ i * 7 + 5], out[ i * 7 + 6]);
		}
		fclose(pf);
	}

	if (1 == theSaveImage)
	{
		cvZero(output);
		for(int i=0;i<n;i++)
		{
			cvLine(output,cvPoint(out[ i * 7 + 0],out[ i * 7 + 1]),cvPoint(out[ i * 7 + 2],out[ i * 7 + 3]),CV_RGB(255,255,255),1, CV_AA);
		}
	}

	/* free memory */
	free( (void *) image );
	free( (void *) out );

	return true;
}