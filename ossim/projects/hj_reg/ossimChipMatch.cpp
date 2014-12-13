// class ossimChipMatch implementation
// REQUIRES FFTW version 3.x (Fast Fourier Transform)

#include "ossimChipMatch.h"
#include <ossim/imaging/ossimImageDataFactory.h>
#include <ossim/base/ossimIrect.h>
#include "ossimRunningSum.h"
#include "ossimNCC_FFTW.h"
#include <ossim/projection/ossimProjection.h>
#include <ossim/projection/ossimProjectionFactoryRegistry.h>
#include <ossim/projection/ossimLandSatModel.h>
#include <fftw3.h>

#include <iostream> //TBR
static int ii=0;
RTTI_DEF1( ossimChipMatch, "ossimChipMatch", ossimImageCombiner );

// matrix to get the 2nd order x,y best fit polynomial (least mean squares)
// -order of values (inputs) : from top left to bottom right along rows (normal image scan)
// -order of coefficients (results) : 1 x y xy xx yy
// uniform weighting for least mean squares

double ossimChipMatch::theLMS[6*9] = {
-1.1111111111111116e-001,2.2222222222222210e-001,-1.1111111111111116e-001,2.2222222222222210e-001,5.5555555555555536e-001,2.2222222222222210e-001,-1.1111111111111116e-001,2.2222222222222210e-001,-1.1111111111111116e-001,
-1.6666666666666666e-001,0.0000000000000000e+000,1.6666666666666666e-001,-1.6666666666666666e-001,0.0000000000000000e+000,1.6666666666666666e-001,-1.6666666666666666e-001,0.0000000000000000e+000,1.6666666666666666e-001,
-1.6666666666666666e-001,-1.6666666666666666e-001,-1.6666666666666666e-001,0.0000000000000000e+000,0.0000000000000000e+000,0.0000000000000000e+000,1.6666666666666666e-001,1.6666666666666666e-001,1.6666666666666666e-001,
2.5000000000000000e-001,0.0000000000000000e+000,-2.5000000000000000e-001,0.0000000000000000e+000,0.0000000000000000e+000,0.0000000000000000e+000,-2.5000000000000000e-001,0.0000000000000000e+000,2.5000000000000000e-001,
1.6666666666666669e-001,-3.3333333333333331e-001,1.6666666666666669e-001,1.6666666666666674e-001,-3.3333333333333326e-001,1.6666666666666674e-001,1.6666666666666669e-001,-3.3333333333333331e-001,1.6666666666666669e-001,
1.6666666666666669e-001,1.6666666666666674e-001,1.6666666666666669e-001,-3.3333333333333331e-001,-3.3333333333333326e-001,-3.3333333333333331e-001,1.6666666666666669e-001,1.6666666666666674e-001,1.6666666666666669e-001
};

ossimChipMatch::ossimChipMatch()
   :ossimImageCombiner(),
   theSlaveAccuracy(7.0), //TBC
   theMRadius(5), //TBC
   theBias(0.0,0.0),
   theMinNCC(0.75),
   theNCCengine(NULL),
   theTile(NULL)
{
}

ossimChipMatch::ossimChipMatch(ossimObject *owner,         
                               ossimImageSource *centers,
                               ossimImageSource *master, 
                               ossimImageSource *slave   )
   :ossimImageCombiner(owner,
   3,
   0,
   true,
   false),
   theSlaveAccuracy(7.0), //TBC: set to 0
   theMRadius(5), //TBC
   theBias(0.0,0.0),
   theMinNCC(0.75),
   theNCCengine(NULL),
   theTile(NULL)
{
   connectMyInputTo(0, centers);
   connectMyInputTo(1, master);
   connectMyInputTo(2, slave);
}

ossimChipMatch::~ossimChipMatch()
{
   //no need to erase theTile (automatic)
   if (theNCCengine!=NULL)
   {
      delete theNCCengine;
      theNCCengine=NULL;
   }
}

bool
ossimChipMatch::canConnectMyInputTo(ossim_int32 index,const ossimConnectableObject* object)const
{
   //check that inputs just have one band
   //max 3 inputs
   //TBD: check scalar types
   switch(index)
   {
   case 0:
   case 1:
   case 2:
      {
      ossimImageSource* imagep=PTR_CAST(ossimImageSource, object);
      if (imagep)
      {
         //return (imagep->getNumberOfOutputBands()==1);
         return true;
      } else {
         return false;
      }
      }
      break;
   default:
      return false;
   }
}

ossimScalarType
ossimChipMatch::getOutputScalarType()const
{
   return OSSIM_NORMALIZED_DOUBLE;
}

ossimRefPtr<ossimImageData>
ossimChipMatch::getTile(const ossimIrect &rect, ossim_uint32 resLevel)
{
   if(!theTile)
   {
      initialize();
   }
   long w = rect.width();
   long h = rect.height();
   long tw = theTile->getWidth();
   long th = theTile->getHeight();

   theTile->setImageRectangle(rect);
      
   if(w*h != tw*th)
   {
     theTile->initialize();
   }
   else
   {
      theTile->makeBlank();
   }
   
   if (runMatch(rect, resLevel))
   {
      //rasterize tie-points
      ossim_float64* tilebuf = static_cast<ossim_float64*>(theTile->getBuf(0));
      vector<ossimTDpt>::const_iterator it = theTies.begin();
      int xc,yc;
      double x0=rect.ul().x;
      double y0=rect.ul().y;
      for (;it!=theTies.end();++it)
      {
         //remove upperleft corner and round
         xc=(int)std::floor(it->x - x0 + 0.5); //TBC : overflow possible?
         yc=(int)std::floor(it->y - y0 + 0.5);
         tilebuf[ w*yc + xc] = it->score;
      }
      theTile->validate();
   }
   return theTile;
}

// returned vector is destroyed after subsequent calls to getFeatures
const vector<ossimTDpt>&
ossimChipMatch::getFeatures(const ossimIrect &rect, ossim_uint32 resLevel)
{
	//	 ossimImageSource* master = PTR_CAST(ossimImageSource, getInput(1)); //use corner input only
 //    ossimImageSource* slave  = PTR_CAST(ossimImageSource, getInput(2));
	// ossimIrect recttemp1=master->getBoundingRect();
 //    ossimIrect recttemp2=slave->getBoundingRect();
	// ossimIrect temp;
	// temp=recttemp2.clipToRect(recttemp1);
	// /*recttemp1.print(cout);
	// cout<<endl;
	// recttemp2.print(cout);
	// cout<<endl;
	// temp.print(cout);
	// cout<<endl;*/
	// /*ossimKeywordlist kwlM,kwlS;
 //    master->getImageGeometry(kwlM);
 //    slave->getImageGeometry(kwlS);
 //    cout<<kwlM<<endl;
 //    cout<<kwlS<<endl;
 //    ossimProjection* proM = ossimProjectionFactoryRegistry::instance()->createProjection(kwlM);
 //    ossimProjection* proS = ossimProjectionFactoryRegistry::instance()->createProjection(kwlS);
	// recttemp1.print(cout);
	// cout<<endl;
	// recttemp2.print(cout);
	// cout<<endl;*/
 //  ossimKeywordlist kwlS;
 //   slave->getImageGeometry(kwlS);//cout<<geom<<endl;
 //   ossimProjection* proS = ossimProjectionFactoryRegistry::instance()->createProjection(kwlS);
	////proS->print(cout);
	///*ossimIrect temp1(50,50,550,550);
	//ossimGpt sgptl,sgptr;
	//ossimDpt dsgptl,dsgptr;
 //  proS->lineSampleToWorld(temp1.ul(),sgptl);
 //  proS->lineSampleToWorld(temp1.lr(),sgptr);
 //  dsgptl=proS->forward(sgptl);
 //  dsgptr=proS->forward(sgptr);
 //  sgptl.print(cout);
 //  cout<<endl;
 //  sgptr.print(cout);
 //  cout<<endl;
 //  dsgptl.print(cout);
 //  cout<<endl;
 //  dsgptr.print(cout);
 //  cout<<endl;*/
	// if (rect.completely_within(temp))
	// {
 //     runMatch(rect, resLevel);
	// }
	//
 //  return theTies;

   runMatch(rect, resLevel);
   return theTies;
}

void 
ossimChipMatch::initialize()
{
   ossimImageCombiner::initialize();
   theTile = ossimImageDataFactory::instance()->create(this, getOutputScalarType(), getNumberOfOutputBands(), getTileWidth(), getTileHeight());
   theTile->initialize();
   //TBD: change tile creation so it isn't within initialize 
}

double
ossimChipMatch::getNullPixelValue(ossim_uint32 /*band*/)const
{
   return ossim::defaultNull(getOutputScalarType());
}

double
ossimChipMatch::getMinPixelValue(ossim_uint32 /*band*/)const
{
   return ossim::defaultMin(getOutputScalarType());
}

double 
ossimChipMatch::getMaxPixelValue(ossim_uint32 /*band*/)const
{
      return ossim::defaultMax(getOutputScalarType());
}

void 
ossimChipMatch::setProperty(ossimRefPtr<ossimProperty> property)
{
   if(property->getName() == OSSIM_CHIPMATCH_PIXELRADIUS_PROPNAME)
   {
      //take ceiling of floating point value
      setMasterRadius((ossim_uint32)(property->valueToString().toDouble()));//std::ceil(dynamic_cast<const ossimNumericProperty*>(property.get())->asFloat64())));
   } 
   else if (property->getName() == OSSIM_CHIPMATCH_SLAVEACCURACY_PROPNAME)
   {
      setSlaveAccuracy(property->valueToString().toDouble());//dynamic_cast<const ossimNumericProperty*>(property.get())->asFloat64());
   }
   else if (property->getName() == OSSIM_CHIPMATCH_MINNCC_PROPNAME)
   {
      setMinNCC(property->valueToString().toDouble());//property.get())->asFloat64());
   }
   else
   {
      ossimImageCombiner::setProperty(property);
   }
}

ossimRefPtr<ossimProperty>
ossimChipMatch::getProperty(const ossimString& name)const
{
   ossimRefPtr<ossimProperty> resultProperty;
   if(name == OSSIM_CHIPMATCH_PIXELRADIUS_PROPNAME)
   {
      ossimNumericProperty* prProperty = new ossimNumericProperty(name,ossimString::toString(getMasterRadius()));
      prProperty->setConstraints(0.0,50.0); //TBD: change to constants
      prProperty->setCacheRefreshBit();
      resultProperty = prProperty;
   }
   else if (name == OSSIM_CHIPMATCH_SLAVEACCURACY_PROPNAME)
   {
      ossimNumericProperty* prProperty = new ossimNumericProperty(name,ossimString::toString(getSlaveAccuracy()));
      prProperty->setConstraints(0.0,100000.0); //TBD: change to constants
      prProperty->setCacheRefreshBit();
      resultProperty = prProperty;
   }
   else if(name == OSSIM_CHIPMATCH_MINNCC_PROPNAME)
   {
      ossimNumericProperty* prProperty = new ossimNumericProperty(name,ossimString::toString(getMinNCC()));
      prProperty->setConstraints(-1.0,1.0); //TBD: change to constants
      prProperty->setCacheRefreshBit();
      resultProperty = prProperty;
   }
   else
   {
      resultProperty = ossimImageCombiner::getProperty(name);
   }
   return resultProperty;
}

void 
ossimChipMatch::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
   ossimImageCombiner::getPropertyNames(propertyNames);
   
   propertyNames.push_back(OSSIM_CHIPMATCH_PIXELRADIUS_PROPNAME);
   propertyNames.push_back(OSSIM_CHIPMATCH_SLAVEACCURACY_PROPNAME);
   propertyNames.push_back(OSSIM_CHIPMATCH_MINNCC_PROPNAME);
}

bool ossimChipMatch::saveState(ossimKeywordlist& kwl,
                                      const char* prefix)const
{
   
   kwl.add(prefix,
           OSSIM_CHIPMATCH_SLAVEACCURACY_PROPNAME,
           ossimString::toString(getSlaveAccuracy()).c_str(),
           true);

   kwl.add(prefix,
           OSSIM_CHIPMATCH_PIXELRADIUS_PROPNAME,
           ossimString::toString(getMasterRadius()).c_str(),
           true);

   kwl.add(prefix,
           OSSIM_CHIPMATCH_MINNCC_PROPNAME,
           ossimString::toString(getMinNCC()).c_str(),
           true);
   
   return ossimImageCombiner::saveState(kwl,
                                        prefix);
}

bool ossimChipMatch::loadState(const ossimKeywordlist& kwl,
                                      const char* prefix)
{
   const char* sa = kwl.find(prefix, OSSIM_CHIPMATCH_SLAVEACCURACY_PROPNAME);
   if(sa)
   {
      setSlaveAccuracy(ossimString(sa).toDouble());
   }

   const char* pr = kwl.find(prefix, OSSIM_CHIPMATCH_PIXELRADIUS_PROPNAME);
   if(pr)
   {      
      setMasterRadius(ossimString(pr).toUInt32());
   }

   const char* mn = kwl.find(prefix, OSSIM_CHIPMATCH_MINNCC_PROPNAME);
   if(mn)
   {
      setMinNCC(ossimString(mn).toDouble());
   }

   bool result = ossimImageCombiner::loadState(kwl,
                                               prefix);
   
   return result;
}

ossim_uint32
ossimChipMatch::getNumberOfOutputBands()const
{
   return 1;
}

ossimIrect
ossimChipMatch::getBoundingRect(ossim_uint32 resLevel)const
{
   //intersect master+slave and add buffer (accuracy)
   ossimImageSource* master = PTR_CAST(ossimImageSource, getInput(1)); //use corner input only
   ossimImageSource* slave  = PTR_CAST(ossimImageSource, getInput(2)); //use corner input only
   
   ossimIrect res;

   if (master && slave)
   {
     //Intersect rectangles
     res = master->getBoundingRect(resLevel);
     res.clipToRect(slave->getBoundingRect(resLevel));
     //buffer with accuracy in pixels
     //accuracy independent from resLevel
     ossim_int32 iacc = (ossim_int32)ceil(getSlaveAccuracy());
     ossimIpt delta_ur(iacc, -iacc); //image axes used

     res.set_ur(res.ur() + delta_ur);
     res.set_ll(res.ll() - delta_ur);

   } else {
     res.makeNan();
   }
   return res;
}

ossim_uint32
ossimChipMatch::getTileWidth() const
{
   ossimImageSource* input = PTR_CAST(ossimImageSource, getInput(0));
   if(input)
   {
      return input->getTileWidth();
   }   
   return 0;
}

ossim_uint32
ossimChipMatch::getTileHeight() const
{
   ossimImageSource* input = PTR_CAST(ossimImageSource, getInput(0));
   if(input)
   {
      return input->getTileHeight();
   }   
   return 0;
}

void
ossimChipMatch::getDecimationFactor(ossim_uint32 resLevel,
                                          ossimDpt& result)const
{
   ossimImageSource* input = PTR_CAST(ossimImageSource,
                                               getInput(0));
   if(input)
   {
      input->getDecimationFactor(resLevel, result);
   }
}

void
ossimChipMatch::getDecimationFactors(vector<ossimDpt>& decimations)const
{
   ossimImageSource* input = PTR_CAST(ossimImageSource,
                                               getInput(0));
   if(input)
   {
      input->getDecimationFactors(decimations);
   }     
}

ossim_uint32
ossimChipMatch::getNumberOfDecimationLevels()const
{
   ossimImageSource* input = PTR_CAST(ossimImageSource,
                                               getInput(0));
   if(input)
   {
      return input->getNumberOfDecimationLevels();
   } 
   return 0;
}

bool
ossimChipMatch::runMatch(const ossimIrect &rect, ossim_uint32 resLevel)
{
   //erase stored tie points
   theTies.clear();

   //get Inputs
   ossimImageSource* corner = PTR_CAST(ossimImageSource, getInput(0));
   ossimImageSource* master = PTR_CAST(ossimImageSource, getInput(1));
   ossimImageSource* slave  = PTR_CAST(ossimImageSource, getInput(2));
   if (!corner || !master || !slave)
   {
      return false;
   }

   long w = rect.width();
   long h = rect.height();
   ////////////////////////wwadd/////////////
   //get corner data tile (same size as inner tile)
   
   ////	ossimKeywordlist kwlM,kwlS,temp;
   //TBD: use pixel size in meters to change delta_lr according to zoom

	ossimRefPtr<ossimImageGeometry> masterGeom=master->getImageGeometry();
	ossimRefPtr<ossimImageGeometry> slaveGeom=slave->getImageGeometry();

	//ossimString slavename=slave->getLongName();
	//ossimLandSatModel* imagep=PTR_CAST(ossimLandSatModel,slaveGeom->getProjection());
//	ossimRefPtr<ossimFfL7> theFfHdr;
	//theFfHdr = new ossimFfL5(file.c_str());
	// ossimLandSatModel* model = new ossimLandSatModel (*theFfHdr);
	// model->
	//ossimUtmProjection* utmM= new ossimUtmProjection;
	//ossimUtmProjection* utmS= new ossimUtmProjection;
	//utmM->loadState(kwlM);
	//utmS->loadState(kwlS);


	////////////////////////////////////////////////////////////////////////
	////ossimDpt sdptl,sdptr,enl,enr,lsl,lsr;
	//utmM->lineSampleToEastingNorthing(rect.ul(),enl);
	//utmM->lineSampleToEastingNorthing(rect.lr(),enr);
 //   mgptl=utmM->inverse(enl);
	//mgptr=utmM->inverse(enr);
	//
	//sdptl=utmS->forward(mgptl);
	//
	//sdptr=utmS->forward(mgptr);
	//
	//utmS->eastingNorthingToLineSample(sdptl,lsl);
	////cout<<"xiangsuzuobiao="<<lsl<<endl;
	//utmS->eastingNorthingToLineSample(sdptr,lsr);
	////TmM->lineSampleToEastingNorthing(rect.ul(),enl);
	////TmS->lineSampleToEastingNorthing(rect.lr(),enr);
 ////   mgptl=TmM->inverse(enl);
	////mgptr=TmM->inverse(enr);
	////
	////sdptl=TmS->forward(mgptl);
	////
	////sdptr=TmS->forward(mgptr);
	////
	////TmS->eastingNorthingToLineSample(sdptl,lsl);
	//////cout<<"xiangsuzuobiao="<<lsl<<endl;
	////TmS->eastingNorthingToLineSample(sdptr,lsr);



		ossimProjection* MasterProj=	 masterGeom->getProjection();
		ossimProjection* SlaveProj=slaveGeom->getProjection();
	//ossimLandSatModel* SlaveProj=	 PTR_CAST(ossimLandSatModel,slaveGeom->getProjection());
    ossimGpt mgptl,mgptr;
	ossimDpt lsl,lsr;
	MasterProj->lineSampleToWorld(rect.ul(),mgptl);
	SlaveProj->worldToLineSample(mgptl,lsl);
	MasterProj->lineSampleToWorld(rect.lr(),mgptr);
	SlaveProj->worldToLineSample(mgptr,lsr);

	ossimIrect rect1(0,0,0,0),slaveIrect(0,0,0,0);
	//cout<<ii++<<endl;
	slaveIrect=slave->getBoundingRect(0);
	//if ((lsl.x>0)&&lsl.y>0)
	if(	slaveIrect.pointWithin(lsl) && slaveIrect.pointWithin(lsr))
	{
        rect1.set_ulx (lsl.x);
		rect1.set_uly (lsl.y);
		rect1.set_lrx (lsr.x);
		rect1.set_lry (lsr.y);
	}
	else{
		//rect1=rect;
		return false;
	}

	ossimIpt cort;



   /////////////////////////////////////////////////////////////////
   //get corner data tile (same size as inner tile)
   ossimRefPtr<ossimImageData> cornerData = corner->getTile(rect, resLevel);   
   if(!cornerData.valid() || !isSourceEnabled())
   {
      return false;
   }
   
   //TBD: use pixel size in meters to change delta_lr according to zoom

   if((cornerData->getDataObjectStatus() != OSSIM_NULL) && (cornerData->getDataObjectStatus() != OSSIM_EMPTY))
   {                  
      //loop on corners (<>NULL & >=2 TBC)
      ossim_uint32 coff=0; //offset (speedup)
      ossim_int32 ci=0;
      ossim_int32 cj=0;
      //chip image radii
      ossimIpt delta_mr(getMasterRadius(), getMasterRadius());
      ossimIpt delta_lr(getMasterRadius() + (ossim_int32)(ceil(theSlaveAccuracy)),
                        getMasterRadius() + (ossim_int32)(ceil(theSlaveAccuracy)) );
      for(cj=0;cj<h;++cj) //rows
      {
         for(ci=0;ci<w;++ci) //cols
         {
            if (!cornerData->isNull(coff,0))
            {
               //get master data for specified center + radius
               // radius doesn't change with resLevel
               ossimIpt delta_mc(ci,cj);
               ossimIrect mrect(rect.ul()+delta_mc-delta_mr, rect.ul()+delta_mc+delta_mr); //square, size 2*radius+1 pixels
               ossimRefPtr<ossimImageData> masterData = master->getTile(mrect, resLevel); //same resLevel?? TBC
               if ((masterData != NULL)
                   && (masterData->getDataObjectStatus() != OSSIM_EMPTY) 
                   && (masterData->getDataObjectStatus() != OSSIM_PARTIAL))
               {

				   ///////////////////////////////////////
				   	MasterProj->lineSampleToWorld(rect.ul()+delta_mc,mgptl);
					SlaveProj->worldToLineSample(mgptl,lsl);
					if(	slaveIrect.pointWithin(lsl))
					{
						 
						rect1.set_ulx (lsl.x);
						rect1.set_uly (lsl.y);
						rect1.set_lrx (lsr.x);
						rect1.set_lry (lsr.y);
					}
					else{
						continue;
						
						}

                  //get slave data with bias & extended frame (use accuracy)
                  //bias & extension change with scale
                //  ossimIpt delta_sc(ci+(ossim_int32)floor(theBias.x+0.5),cj+(ossim_int32)floor(theBias.y+0.5)); //biased center : TBD : convert unit to pixels
				    ossimIpt delta_sc((ossim_int32)floor(theBias.x+0.5),(ossim_int32)floor(theBias.y+0.5)); //biased center : TBD : convert unit to pixels

                // ossimIrect srect(rect.ul()+delta_sc-delta_lr, rect.ul()+delta_sc+delta_lr); //square, size 2*(radius+accuracy)+1 pixels
				  ossimIrect srect1(lsl-delta_lr, lsl+delta_lr); //wwadd
                 ossimRefPtr<ossimImageData> slaveData = slave->getTile(srect1, resLevel); //same resLevel?? TBC
				  
                  if ((slaveData != NULL) 
                      && (slaveData->getDataObjectStatus() != OSSIM_EMPTY)
                      && (slaveData->getDataObjectStatus() != OSSIM_PARTIAL))
                  {
                     //find normalized cross-correlation maximum
                     //TBD: assuming floating point input
                     double dx=0.0;
                     double dy=0.0;
                     double ncor=0.0;
					 
               //      getMaxCorrelation(masterData, slaveData, &dx, &dy, &ncor);
				//	 cout <<dx <<"   "<<dy<<"   "<<ncor<<endl;
				//	 dx=0.0;dy=0.0;ncor=0.0;
				//	  ossimRefPtr<ossimImageData> slaveData1 = slave->getTile(srect1, resLevel); //same resLevel?? TBC
                    getMaxCorrelation(masterData, slaveData, &dx, &dy, &ncor);
				//	 cout <<dx <<"   "<<dy<<"   "<<ncor<<endl;
                     //filter on NCC value
                     if (ncor >= theMinNCC)
                     {
                        //create tie point & store
						// ossimDpt offset=rect1.ul()+delta_sc-rect.ul()-delta_mc;
						 ossimDpt dxy;
						// dxy=dxy+offset;
					//	 dxy.x=rect1.ul().x+delta_sc.x-delta_lr.x+(dx+delta_lr.x+delta_mr.x-(2*delta_mr.x));
					//	 dxy.y=rect1.ul().y+delta_sc.y-delta_lr.y+(dy+delta_lr.y+delta_mr.y-(2*delta_mr.y));
						 dxy.x=lsl.x+delta_sc.x+dx;
						dxy.y=lsl.y+delta_sc.y+dy;
                        theTies.push_back(ossimTDpt( rect.ul()+delta_mc, dxy, ncor ));
                     }
                  }
               }
			}
            ++coff;
		 }
      }


      return true;
   }
   return false;
}

void
ossimChipMatch::getMaxCorrelation(ossimRefPtr<ossimImageData> Mchip, ossimRefPtr<ossimImageData> Schip, 
                                  double* pdispx, double* pdispy, double* pcor)
{
   //use FFTW 3.0.1
   //assume displacement between center of master to center of slave buffer
   // Mchip must smaller than Schip (Schip incorporates error buffer)

   ossim_uint32 sx = Schip->getWidth();
   ossim_uint32 sy = Schip->getHeight();
   ossim_uint32 mx = Mchip->getWidth();
   ossim_uint32 my = Mchip->getHeight();
   //cout<<"mx="<<mx<<" my="<<my<<" sx="<<sx<<" sy="<<sy<<endl; //TBR

   int cx=sx+mx-1;
   int cy=sy+my-1;

   if (theNCCengine!=NULL)
   {
      //check correlation size
      if (!theNCCengine->sameDims(cy,cx))
      {
         //re build NCC engine //TBD : use wisdom
         delete theNCCengine;
         theNCCengine=NULL;
      }
   }
   if (theNCCengine==NULL)
   {
      //build a new NCC engine //TBD : use wisdom
      theNCCengine = new ossimNCC_FFTW(cy,cx);
   }

   theNCCengine->ingestMaster(my,mx,Mchip->getDoubleBuf());
   theNCCengine->ingestSlave(sy,sx,Schip->getDoubleBuf());

   if (!theNCCengine->calculateNCC())
   {
      // TBD err mngt
      if (pcor) *pcor=0.0;
      if (pdispx) *pdispx = 0.0;
      if (pdispy) *pdispy = 0.0;
      cout<<"Error in NCC calculation"<<endl;
      return;
   }
   int mj          = theNCCengine->getMaxCorrX(); 
   int mi          = theNCCengine->getMaxCorrY();
   double bestcorr = theNCCengine->getMaxCorr();
   int oj = (cx-1)/2;//we know that cx and cy are odd!!
   int oi = (cy-1)/2;
   int deltaj = (sx-mx)/2; //we know that sx-mx is even
   int deltai = (sy-my)/2;

   //original best shift (integer shift for for max value)
   double dmcx = mj - oj;
   double dmcy = mi - oi;

   //find maximum, sub-pixel precision
   //use least-square fit on 2nd order polynomial
   if ((mj > oj-deltaj) && (mj < oj+deltaj) && (mi > oi-deltai) && (mi < oi+deltai))
   {
      //then there's a 3x3 neighborhood we can use to get better precision
      vector<double> p2c(6); //2nd order x y polynomial coefficents (see theLMS comments)
      vector<double>::iterator it = p2c.begin();
      double* pm = theLMS;
      const ossimNCC_FFTW::cMatrix& corrmat = theNCCengine->getNcc();
      //matrix product with values of 3x3 neighborhood
      for (int k=0;k<6;++k)
      {
         *it = 0.0;
         for(int i=-1;i<=1;++i)
         {
            for(int j=-1;j<=1;++j)
            {
               *it += *(pm++) * corrmat(mi+i, mj+j);
            }
         }
         ++it;
      }
      //check convexity (det>0) + downwards orientation (trace<0)
      double trace = p2c[4] + p2c[5];
      if (trace<-1e-13) //TBC : -epsilon
      {
         double det = p2c[4]*p2c[5] - 0.25*p2c[3]*p2c[3];
         if (det>1e-13) //TBC : epsilon
         {
            //ok : convex + downwards
            //find maximum position
            double optx = (p2c[3]*p2c[2] - 2.0 * p2c[5]*p2c[1]) / det * 0.25;
            double opty = (p2c[3]*p2c[1] - 2.0 * p2c[4]*p2c[2]) / det * 0.25;
            //limit new position to center pixel square
            //TBD : need to find better model for NCC subpixel
            if ((fabs(optx)<=0.501) && (fabs(opty)<=0.501))
            {
               dmcx+=optx;
               dmcy+=opty;
              //change corelation max value (dangerous) : TBD ? TBC
            }
         }
      }
   }

   //give results back  
   if (pcor)   *pcor   = bestcorr;
   if (pdispx) *pdispx = dmcx;
   if (pdispy) *pdispy = dmcy;
}

