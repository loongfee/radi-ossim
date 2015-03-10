//*****************************************************************************
// FILE: ossimSensorModel.cc
//
// License:  LGPL
// 
// See LICENSE.txt file in the top level directory for more details.
//
// AUTHOR: Oscar Kramer
//
// DESCRIPTION:
//   Contains implementation of class ossimSensorModel. This is the base
//   class to all sensor model-related projections including replacement
//   models such as coarse grids and polynomial models. This base class
//   supports adjustable parameters for registration adjustment.
//
//   Important note to sensor model implementors: In order to avoid a separate
//   set of "initial adjustable parameters," this design assumes ALL initial
//   values are 0. When designing the derived-class model, insure that the
//   adjustable parameters are 0-based. This applies to the adjustable param
//   array declared in this base class only. The derived classes can declare
//   their own adjstable params that are more descriptive and that can be
//   assigned an initial value thet is non-zero, but that are linearly related
//   to the adjustable params of this class. In that case, the updateModel()
//   method will compute the derived-class's parameters based on
//   theAdjustableParams array, after an adjustment is made.
//
// LIMITATIONS: None.
//
//*****************************************************************************
//  $Id: ossimSensorModel.cpp 21808 2012-10-05 14:23:33Z dhicks $
#include <iostream>
#include <sstream>
using namespace std;

// #include <stdio.h>

#include <ossim/projection/ossimSensorModel.h>
///////////////begin ww1130
#include <ossim/base/ossimTempFilename.h>
#include <ossim/projection/ossimMapProjectionFactory.h>
#include <ossim/projection/ossimProjectionFactoryRegistry.h>
//////////////////////////////////////end
RTTI_DEF3(ossimSensorModel, "ossimSensorModel", ossimProjection, ossimOptimizableProjection, ossimAdjustableParameterInterface);

#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimDrect.h>
#include <ossim/base/ossimDatum.h>
#include <ossim/base/ossimException.h>
#include <ossim/base/ossimNotifyContext.h>
#include <ossim/base/ossimDatumFactory.h>

#include <ossim/elevation/ossimElevManager.h>
#include <ossim/base/ossimTieGptSet.h>

#include <ossim/matrix/newmatrc.h>
//#include <mlpack/methods/lars/lars.hpp>
//using namespace arma;
//using namespace std;
//using namespace mlpack;
//using namespace mlpack::regression;

//***
// Define Trace flags for use within this file:
//***
#include <ossim/base/ossimTrace.h>
static ossimTrace traceExec  ("ossimSensorModel:exec");
static ossimTrace traceDebug ("ossimSensorModel:debug");

static const char*       REF_GPT_LAT_KW      = "ref_point_lat";
static const char*       REF_GPT_LON_KW      = "ref_point_lon";
static const char*       REF_GPT_HGT_KW      = "ref_point_hgt";
static const char*       REF_IPT_LINE_KW     = "ref_point_line";
static const char*       REF_IPT_SAMP_KW     = "ref_point_samp";
static const char*       IMAGE_ID_KW         = "image_id";
static const char*       SENSOR_ID_KW        = "sensor";
static const char*       IMAGE_DATE_KW        = "image_acquisitiondate"; 
static const ossimString NULL_STRING         = "NULL";
static const double      RAY_ORIGIN_HEIGHT   = 10000.0; //meters


//DEBUG TBR : output ops
std::ostream& operator<<(std::ostream& os, NEWMAT::GeneralMatrix& mat)
{
   int nr=mat.Nrows();
   int nc=mat.Ncols();

   NEWMAT::MatrixRow crow(&mat,NEWMAT::LoadOnEntry);
//   NEWMAT::MatrixRowCol crow;
   mat.RestoreRow(crow);  
   for (int r=0;r<nr;++r)
   {
      for (int c=0;c<nc;++c)
      {
         os<<*(crow.Data()+c)<<" ";
      }
      os<<std::endl;
      mat.NextRow(crow);
   }
   return os;
}

//*****************************************************************************
//  DEFAULT CONSTRUCTOR: ossimSensorModel()
//*****************************************************************************
ossimSensorModel::ossimSensorModel()
   :
   ossimOptimizableProjection       (),
   ossimAdjustableParameterInterface(),
   theImageSize        (0, 0),
   theSubImageOffset   (0.0, 0.0),
   theImageID          (),
   theSensorID         (),
   theGSD              (0.0, 0.0),
   theMeanGSD          (0.0),
   theRefGndPt         (0.0, 0.0, 0.0),
   theRefImgPt         (0.0, 0.0),
   theBoundGndPolygon  (),
   theImageClipRect    (),
   theRelPosError      (0),
   theNominalPosError  (0),
   theExtrapolateImageFlag(false),
   theExtrapolateGroundFlag(false)
{
	   m_proj=NULL;
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::ossimSensorModel(geom_kwl): entering..." << endl;

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::ossimSensorModel(geom_kwl): returning..." << std::endl;
}

//*****************************************************************************
//  COPY CONSTRUCTOR: ossimSensorModel(ossimSensorModel)
//*****************************************************************************
ossimSensorModel::ossimSensorModel(const ossimSensorModel& model)
   :
   ossimOptimizableProjection(model),
   ossimAdjustableParameterInterface(model),
   theImageSize       (model.theImageSize),
   theSubImageOffset  (model.theSubImageOffset),
   theImageID         (model.theImageID),
   theSensorID        (model.theSensorID),
   theGSD             (model.theGSD),
   theMeanGSD         (model.theMeanGSD),
   theRefGndPt        (model.theRefGndPt),
   theRefImgPt        (model.theRefImgPt),
   theBoundGndPolygon (model.theBoundGndPolygon),
   theImageClipRect   (model.theImageClipRect),
   theRelPosError      (model.theRelPosError),
   theNominalPosError (model.theNominalPosError),
   theExtrapolateImageFlag(false),
   theExtrapolateGroundFlag(false)
{
	   m_proj = model.m_proj;
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::ossimSensorModel(model): entering..." << std::endl;

   theErrorStatus = model.theErrorStatus;
   
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::ossimSensorModel(model): returning..." << std::endl;
   return;
}

//*****************************************************************************
//  CONSTRUCTOR: ossimSensorModel(geom_kwl)
//  
//  Constructs from a geometry keywordlist.
//  
//*****************************************************************************
ossimSensorModel::ossimSensorModel(const ossimKeywordlist& geom_kwl)
   :
   ossimOptimizableProjection     (),
   ossimAdjustableParameterInterface(),
   theImageSize        (0, 0),
   theSubImageOffset   (0.0, 0.0),
   theImageID          (),
   theSensorID         (),
   theGSD              (0.0, 0.0),
   theMeanGSD          (0.0),
   theRefGndPt         (0.0, 0.0, 0.0),
   theRefImgPt         (0.0, 0.0),
   theBoundGndPolygon  (),
   theImageClipRect    (),
   theRelPosError      (0),
   theNominalPosError      (0),
   theExtrapolateImageFlag(false),
   theExtrapolateGroundFlag(false)
{
	   m_proj=NULL;
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::ossimSensorModel(geom_kwl): entering..." << std::endl;

//   ossimElevManager::instance()->loadState(geom_kwl);
   loadState(geom_kwl);

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::ossimSensorModel(geom_kwl): returning..." << std::endl;
   return;
}

//*****************************************************************************
//  DESTRUCTOR: ~ossimSensorModel
//  
//*****************************************************************************
ossimSensorModel::~ossimSensorModel()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::~ossimSensorModel: entering..." << std::endl;
   
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::~ossimSensorModel: returning..." << std::endl;
}

ossimObject* ossimSensorModel::getBaseObject()
{
   return this;
}

const ossimObject* ossimSensorModel::getBaseObject()const
{
   return this;
}


//*****************************************************************************
//  METHOD: ossimSensorModel::lineSampleToWorld(image_pt, &gpt)
//  
//  Intersects the ray associated with image_pt with the available elevation
//  model. Returns ground point
//  
//*****************************************************************************
void ossimSensorModel::lineSampleToWorld(const ossimDpt& image_point,
                                         ossimGpt&       gpt) const
{
   bool debug = false;  // setable via interactive debugger
   if (traceExec() || debug)  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::lineSampleToWorld:entering..." << std::endl;
   
   if(image_point.hasNans())
   {
      gpt.makeNan();
      return;
   }
   //***
   // Extrapolate if image point is outside image:
   //***
   if (!insideImage(image_point)&&(!theExtrapolateImageFlag))
   {
      gpt = extrapolate(image_point);
      return;
   }

   //***
   // Determine imaging ray and invoke elevation source object's services to
   // intersect ray with terrain model:
   //***
   ossimEcefRay ray;
   imagingRay(image_point, ray);
   ossimElevManager::instance()->intersectRay(ray, gpt);

   if (traceDebug() || debug)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "image_point = " << image_point << std::endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "ray = " << ray << std::endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "gpt = " << gpt << std::endl;
   }

   if (traceExec() || debug)  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::lineSampleToWorld: returning..." << std::endl;
   return;
}

//*****************************************************************************
//  METHOD: ossimSensorModel::worldToLineSample()
//  
//  Performs forward projection of ground point to image space.
//  
//*****************************************************************************
void ossimSensorModel::worldToLineSample(const ossimGpt& worldPoint,
                                         ossimDpt&       ip) const
{
   //   static bool recursionFlag = false;

   static const double PIXEL_THRESHOLD    = .1; // acceptable pixel error
   static const int    MAX_NUM_ITERATIONS = 20;


   if(worldPoint.isLatNan()||
      worldPoint.isLonNan())
     {
       ip.makeNan();
       return;
     }
      
   //***
   // First check if the world point is inside bounding rectangle:
   //***
   int iters = 0;
   ossimDpt wdp (worldPoint);
   //   if ((!recursionFlag)&&!(theBoundGndPolygon.pointWithin(wdp)))

   //if((theBoundGndPolygon.getNumberOfVertices() > 0)&&
   //   (!theBoundGndPolygon.hasNans()))
   //{
   //   if (!(theBoundGndPolygon.pointWithin(wdp)))
   //   {
   //      if(theSeedFunction.valid())
   //      {
   //         theSeedFunction->worldToLineSample(worldPoint, ip);
   //      }
   //      else if(!theExtrapolateGroundFlag) // if I am not already in the extrapolation routine

   //      {
   //      //      recursionFlag = true;
   //         ip = extrapolate(worldPoint);
   //      //      recursionFlag = false;
   //      }
   //      return;
   //   }         
   //}

   //***
   // Substitute zero for null elevation if present:
   //***
   double height = worldPoint.hgt;
   if ( ossim::isnan(height) )
   {
      height = 0.0;
   }

   //
   // Utilize iterative scheme for arriving at image point. Begin with guess
   // at image center:
   //
   if(theSeedFunction.valid())
   {
      theSeedFunction->worldToLineSample(worldPoint, ip);
   }
   else
   {
      ip.u = theRefImgPt.u;
      ip.v = theRefImgPt.v;
   }
   
   ossimDpt ip_du;
   ossimDpt ip_dv;

   ossimGpt gp, gp_du, gp_dv;
   double dlat_du, dlat_dv, dlon_du, dlon_dv;
   double delta_lat, delta_lon, delta_u, delta_v;
   double inverse_norm;
   bool done = false;
   //***
   // Begin iterations:
   //***
   do
   {
      //***
      // establish perturbed image points about the guessed point:
      //***
      ip_du.u = ip.u + 1.0;
      ip_du.v = ip.v;
      ip_dv.u = ip.u;
      ip_dv.v = ip.v + 1.0;
      
      //***
      // Compute numerical partials at current guessed point:
      //***
      lineSampleHeightToWorld(ip,    height, gp);
      lineSampleHeightToWorld(ip_du, height, gp_du);
      lineSampleHeightToWorld(ip_dv, height, gp_dv);

      if(gp.isLatNan() || gp.isLonNan())
      {
         gp = extrapolate(ip);
      }
      if(gp_du.isLatNan() || gp_du.isLonNan())
      {
         gp_du = extrapolate(ip_du);
      }
      if(gp_dv.isLatNan() | gp_dv.isLonNan())
      {
         gp_dv = extrapolate(ip_dv);
         
      }
      dlat_du = gp_du.lat - gp.lat; //e
      dlon_du = gp_du.lon - gp.lon; //g
      dlat_dv = gp_dv.lat - gp.lat; //f
      dlon_dv = gp_dv.lon - gp.lon; //h
      
      //
      // Test for convergence:
      //
      delta_lat = worldPoint.lat - gp.lat;
      delta_lon = worldPoint.lon - gp.lon;


      //
      // Compute linearized estimate of image point given gp delta:
      //
      inverse_norm = dlat_dv*dlon_du - dlat_du*dlon_dv; // fg-eh
      
      if (!ossim::almostEqual(inverse_norm, 0.0, DBL_EPSILON))
      {
         delta_u = (-dlon_dv*delta_lat + dlat_dv*delta_lon)/inverse_norm;
         delta_v = ( dlon_du*delta_lat - dlat_du*delta_lon)/inverse_norm;
         ip.u += delta_u;
         ip.v += delta_v;
      }
      else
      {
         delta_u = 0;
         delta_v = 0;
      }
      done = ((fabs(delta_u) < PIXEL_THRESHOLD)&&
              (fabs(delta_v) < PIXEL_THRESHOLD));
      iters++;
   } while ((!done) &&
             (iters < MAX_NUM_ITERATIONS));
//    } while (((fabs(delta_u) > PIXEL_THRESHOLD) ||
//              (fabs(delta_v) > PIXEL_THRESHOLD)) &&
//             (iters < MAX_NUM_ITERATIONS));

   //***
   // Note that this error mesage appears only if max count was reached while
   // iterating. A linear (no iteration) solution would finish with iters =
   // MAX_NUM_ITERATIONS + 1:
   //***
   if (iters >= MAX_NUM_ITERATIONS)
   {
//       std::cout << "MAX ITERATION!!!" << std::endl;
//       std::cout << "delta_u = "   << delta_u
//                 << "\ndelta_v = " << delta_v << "\n";
   }
   else
   {
//       std::cout << "ITERS === " << iters << std::endl;
   }
//    std::cout << "iters = " << iters << "\n";
   //***
   // The image point computed this way corresponds to full image space.
   // Apply image offset in the case this is a sub-image rectangle:
   //***
      ip -= theSubImageOffset;

   return;
}

//*****************************************************************************
//  METHOD: ossimSensorModel::print(ostream)
//  
//  Dumps contents of object to ostream.
//  
//*****************************************************************************
std::ostream& ossimSensorModel::print(std::ostream& out) const
{
   out << setprecision(15) << setiosflags(ios::fixed)
       << "\n ossimSensorModel base-class data members:\n"
       << "\n         theImageID: " << theImageID
       << "\n        theSensorID: " << theSensorID
       << "\n       theImageSize: " << theImageSize
       << "\n  theSubImageOffset: " << theSubImageOffset
       << "\n             theGSD: " << theGSD
       << "\n         theMeanGSD: " << theMeanGSD
       << "\n        theRefGndPt: " << theRefGndPt
       << "\n        theRefImgPt: " << theRefImgPt
       << "\n theBoundGndPolygon: \n" << theBoundGndPolygon
       << "\n   theImageClipRect: " << theImageClipRect
       << "\n theNominalPosError: " << theNominalPosError
       << "\n     theNominalPosError: " << theNominalPosError
       << "\n     theRelPosError: " << theRelPosError
       << endl;

   //---
   // Note:
   // need ossimAdjustableParameterInterface::print(os);
   //---
   return ossimProjection::print(out);
}

void ossimSensorModel::setRefImgPt(const ossimDpt& pt)
{
   theRefImgPt = pt;
}

void ossimSensorModel::setRefGndPt(const ossimGpt& pt)
{
   theRefGndPt = pt;
}

void ossimSensorModel::setImageRect(const ossimDrect& imageRect)
{
   theImageClipRect = imageRect;
   theRefImgPt = imageRect.midPoint();
}
void ossimSensorModel::setGroundRect(const ossimGpt& ul,
                                     const ossimGpt& ur,
                                     const ossimGpt& lr,
                                     const ossimGpt& ll)
{
   theBoundGndPolygon.clear();
   theBoundGndPolygon.addPoint(ul);
   theBoundGndPolygon.addPoint(ur);
   theBoundGndPolygon.addPoint(lr);
   theBoundGndPolygon.addPoint(ll);
}


//*****************************************************************************
//  METHOD: ossimSensorModel::saveState()
//  
//  Saves the state of this object to KWL.
//  
//*****************************************************************************
bool ossimSensorModel::saveState(ossimKeywordlist& kwl,
                                 const char*       prefix) const 
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::saveState: entering..." << std::endl;

   kwl.add(prefix, IMAGE_ID_KW, theImageID.chars());
   kwl.add(prefix, SENSOR_ID_KW, theSensorID.chars());
   
   kwl.add(prefix,
           ossimKeywordNames::NUMBER_LINES_KW,
           theImageSize.line,
           true);
   kwl.add(prefix,
           ossimKeywordNames::NUMBER_SAMPLES_KW,
           theImageSize.samp,
           true);
   
   kwl.add(prefix,
           REF_GPT_LAT_KW,
           theRefGndPt.lat,
           true);
   
   kwl.add(prefix,
           REF_GPT_LON_KW,
           theRefGndPt.lon,
           true);
   
   kwl.add(prefix,
           REF_GPT_HGT_KW,
           theRefGndPt.hgt,
           true);
   
   kwl.add(prefix,
           REF_IPT_LINE_KW,
           theRefImgPt.line,
           true);
      
   kwl.add(prefix,
           REF_IPT_SAMP_KW,
           theRefImgPt.samp,
           true);
   
   kwl.add(prefix,
           ossimKeywordNames::METERS_PER_PIXEL_Y_KW,
           theGSD.line,
           true);
   
   kwl.add(prefix,
           ossimKeywordNames::METERS_PER_PIXEL_X_KW,
           theGSD.samp,
           true);
   
   ossimDpt corner;
   if(!theBoundGndPolygon.vertex(0, corner))
   {
      corner = ossimDpt(0,0);
   }
   
   kwl.add(prefix,
           ossimKeywordNames::UL_LAT_KW,
           corner.lat,
           true);
   
   kwl.add(prefix,
           ossimKeywordNames::UL_LON_KW,
           corner.lon,
           true);
   
   if(!theBoundGndPolygon.nextVertex(corner))
   {
      corner = ossimDpt(0,0);
   }
   kwl.add(prefix,
           ossimKeywordNames::UR_LAT_KW,
           corner.lat,
           true);
   
   kwl.add(prefix,
           ossimKeywordNames::UR_LON_KW,
           corner.lon,
           true);
   
   if(!theBoundGndPolygon.nextVertex(corner))
   {
      corner = ossimDpt(0,0);
   }
   kwl.add(prefix,
           ossimKeywordNames::LR_LAT_KW,
           corner.lat,
           true);
   kwl.add(prefix,
           ossimKeywordNames::LR_LON_KW,
           corner.lon,
           true);
   
   
   if(!theBoundGndPolygon.nextVertex(corner))
   {
      corner = ossimDpt(0,0);
   }
   kwl.add(prefix,
           ossimKeywordNames::LL_LAT_KW,
           corner.lat,
           true);
   kwl.add(prefix,
           ossimKeywordNames::LL_LON_KW,
           corner.lon,
           true);

   kwl.add(prefix,
           ossimKeywordNames::CE90_ABSOLUTE_KW,
           theNominalPosError,
           true, 20);

   kwl.add(prefix,
           ossimKeywordNames::CE90_RELATIVE_KW,
           theRelPosError,
           true, 20);

   kwl.add(prefix,
           "rect",
           ossimString::toString(theImageClipRect.ul().x)
           + " " + ossimString::toString(theImageClipRect.ul().y) + " " +
           ossimString::toString(theImageClipRect.lr().x) + " " +
           ossimString::toString(theImageClipRect.lr().y),
           true);

   // Avoid passing null char* to method that takes an ossimString.
   ossimString tmpStr;
   if (prefix)
   {
      tmpStr = prefix;
   }
   saveAdjustments(kwl, tmpStr);
      
   if (m_proj)	m_proj->saveState(kwl, "m_proj."); ////// ww1130
   // Also save the state of the elevation object:
   //
//   ossimElevManager::instance()->saveState(kwl, prefix);
   
   
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::saveState:returning..." << std::endl;
   return ossimProjection::saveState(kwl, prefix);;
}

//*****************************************************************************
//  METHOD: ossimSensorModel::loadState()
//  
//  Loads the state of this object from KWL.
//  
//*****************************************************************************
bool ossimSensorModel::loadState(const ossimKeywordlist& kwl,
                                 const char*       prefix)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::loadState: entering..." << std::endl;

   const char* keyword;
   const char* value;
//   int nconv;
   ossimDpt v[4]; // temporarily holds vertices for ground polygon
   keyword = IMAGE_DATE_KW;
   value = kwl.find(prefix, keyword);
   if (value)
      theSensorDATE = value;
   else
      theSensorDATE = NULL_STRING;

   //***
   // Read each keyword, breaking out if error occurs:
   //***
   keyword = IMAGE_ID_KW;
   value = kwl.find(prefix, keyword);
   if (value)
      theImageID = value;
   else
      theImageID = NULL_STRING;
   
   keyword = SENSOR_ID_KW;
   value = kwl.find(prefix, keyword);
   if (value)
      theSensorID = value;
   else
      theSensorID = NULL_STRING;
      
   keyword = ossimKeywordNames::NUMBER_LINES_KW;//IMAGE_SIZE_LINES_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      theImageSize.line = ossimString(value).toLong();
   }

   keyword = ossimKeywordNames::NUMBER_SAMPLES_KW;// IMAGE_SIZE_SAMPS_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      theImageSize.samp = ossimString(value).toLong();
   }

   keyword = REF_IPT_LINE_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      theRefImgPt.line = ossimString(value).toDouble();
   }

   keyword = REF_IPT_SAMP_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      theRefImgPt.samp = ossimString(value).toDouble();
   }

   keyword = REF_GPT_LAT_KW;
   value = kwl.find(prefix, keyword);
   if(value)
   {
      theRefGndPt.latd(ossimString(value).toDouble());
   }
   

   keyword = REF_GPT_LON_KW;
   value = kwl.find(prefix, keyword);
   if(value)
   {
      theRefGndPt.lond(ossimString(value).toDouble());
   }
   

   keyword = REF_GPT_HGT_KW;
   value = kwl.find(prefix, keyword);
   if(value)
   {
      theRefGndPt.hgt = ossimString(value).toDouble();
   }
   

   keyword = ossimKeywordNames::METERS_PER_PIXEL_Y_KW;// GSD_LINE_DIR_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      theGSD.line = ossimString(value).toDouble();
   }

   keyword = ossimKeywordNames::METERS_PER_PIXEL_X_KW;//GSD_SAMP_DIR_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      theGSD.samp = ossimString(value).toDouble();
   }

   theMeanGSD = (fabs(theGSD.line) + fabs(theGSD.samp))/2.0;

   keyword = ossimKeywordNames::UL_LAT_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      v[0].lat = ossimString(value).toDouble();
   }

   keyword = ossimKeywordNames::UL_LON_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      v[0].lon = ossimString(value).toDouble();
   }

   keyword = ossimKeywordNames::UR_LAT_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      v[1].lat = ossimString(value).toDouble();
   }

   keyword = ossimKeywordNames::UR_LON_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      v[1].lon = ossimString(value).toDouble();
   }

   keyword = ossimKeywordNames::LR_LAT_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      v[2].lat = ossimString(value).toDouble();
   }

   keyword = ossimKeywordNames::LR_LON_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      v[2].lon = ossimString(value).toDouble();
   }

   keyword = ossimKeywordNames::LL_LAT_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      v[3].lat = ossimString(value).toDouble();
   }

   keyword = ossimKeywordNames::LL_LON_KW;
   value = kwl.find(prefix, keyword);
   if (value)
   {
      v[3].lon = ossimString(value).toDouble();
   }

   keyword = ossimKeywordNames::CE90_ABSOLUTE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
   {
      // Try old keyword for legacy purposes:
      keyword = ossimKeywordNames::IMAGE_CE90_KW;
      value = kwl.find(prefix, keyword);
   }
   if (value)
      theNominalPosError = atof(value);
   else
      theNominalPosError = 0.0;

   keyword = ossimKeywordNames::CE90_RELATIVE_KW;
   value = kwl.find(prefix, keyword);
   if (value)
      theRelPosError = atof(value);
   else
      theRelPosError = theNominalPosError;

   //***
   // Initialize other data members given quantities read in KWL:
   //***
   theBoundGndPolygon = ossimPolygon(4, v);

   const char* rect = kwl.find(prefix, "rect");
   if(rect)
   {
      std::vector<ossimString> splitArray;
      ossimString rectString(rect);
      rectString = rectString.trim();
      rectString.split(splitArray, " ");
      if(splitArray.size() == 4)
      {
         theImageClipRect = ossimDrect(splitArray[0].toDouble(),
                                       splitArray[1].toDouble(),
                                       splitArray[2].toDouble(),
                                       splitArray[3].toDouble());
      }
      else
      {
         theImageClipRect = ossimDrect(0.0, 0.0,
                                       theImageSize.samp-1, theImageSize.line-1);
      }
   }
   else
   {
      theImageClipRect = ossimDrect(0.0, 0.0,
                                    theImageSize.samp-1, theImageSize.line-1);
   }
   
   // Avoid passing null char* to method that takes an ossimString.
   ossimString tmpStr;
   if (prefix)
   {
      tmpStr = prefix;
   }
   loadAdjustments(kwl, tmpStr);

   /////////////////////////////////begin  ww1130//////////////////
   m_proj=PTR_CAST(ossimMapProjection,
	ossimMapProjectionFactory::instance()->createProjection(kwl,"m_proj."));
   ///////////////////////////////////////////////////////////end
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::loadState: returning..." << std::endl;
   return ossimProjection::loadState(kwl, prefix);;
}

//*****************************************************************************
// PROTECTED METHOD: Model_type::extrapolate(image_point)
//
//  This method computes extrapolated values of latitude and longitude for
//  points which are outside the actual image boundaries.
//*****************************************************************************
ossimGpt ossimSensorModel::extrapolate (const ossimDpt& imagePoint,
                                        const double&   height) const
{
   theExtrapolateImageFlag = true;
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) <<  "DEBUG ossimSensorModel::extrapolate: entering... " << std::endl;

   //---
   // If image point supplied has NaN components, return now with a NaN point.
   // This prevents an infinite recursion between model worldToLineSample
   // and this method:
   //---
   if (imagePoint.hasNans())
   {
      theExtrapolateImageFlag = false;
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::extrapolate: returning..." << std::endl;
      return ossimGpt(ossim::nan(), ossim::nan(), ossim::nan());
   }

   if(theSeedFunction.valid())
   {
      ossimGpt wpt;

      theSeedFunction->lineSampleToWorld(imagePoint, wpt);
      theExtrapolateImageFlag = false;
      return wpt;
   }
   //***
   // Determine which edge is intersected by the radial, and establish
   // intersection:
   //***
   ossimGpt gpt;
   ossimDpt edgePt (imagePoint);
   ossimDpt image_center (theRefImgPt);
   theImageClipRect.clip(image_center, edgePt);

   //***
   // Need edgePt relative to image center. Compute an epsilon perturbation in
   // the direction of edgePt for later computing directional derivative,
   // and back out the offset to origin:
   //***
   ossimDpt deltaPt (edgePt - image_center);
   ossimDpt epsilon (deltaPt/deltaPt.length());
   edgePt -= epsilon;  // insure that we are inside the image
   ossimDpt edgePt_prime (edgePt - epsilon); // epsilon=1pixel
       
   //***
   // Establish ground point corresponding to edge point and edgePt+epsilon:
   //***
   ossimGpt edgeGP;
   ossimGpt edgeGP_prime;

   if (ossim::isnan(height))
   {
      lineSampleToWorld(edgePt, edgeGP);
      lineSampleToWorld(edgePt_prime, edgeGP_prime);
   }
   else
   {
      lineSampleHeightToWorld(edgePt, height, edgeGP);
      lineSampleHeightToWorld(edgePt_prime, height, edgeGP_prime);
   }
   
   //***
   // Compute approximate directional derivatives of lat and lon along radial
   // at the edge:
   //***
   double dpixel    = (edgePt-edgePt_prime).length();
   double dlat_drad = (edgeGP.lat - edgeGP_prime.lat)/dpixel;
   double dlon_drad = (edgeGP.lon - edgeGP_prime.lon)/dpixel;

   //***
   // Now extrapolate to image point of interest:
   //***
   double delta_pixel = (imagePoint - edgePt).length();

   gpt.lat = edgeGP.lat + dlat_drad*delta_pixel;
   gpt.lon = edgeGP.lon + dlon_drad*delta_pixel;
   if ( ossim::isnan(height) )
   {
      gpt.hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(gpt);
   }
   else
   {
      gpt.hgt = height;
   }
   theExtrapolateImageFlag = false;
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::extrapolate: returning..." << std::endl;
   return gpt;
}


//*****************************************************************************
// PROTECTED METHOD: Model_type::extrapolate(ground_point)
//
//  This method computes extrapolated values of line and sample for
//  points which are outside the actual image boundaries.
//*****************************************************************************
ossimDpt ossimSensorModel::extrapolate (const ossimGpt& gpt) const
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) <<  "DEBUG ossimSensorModel::extrapolate: entering... " << std::endl;
   theExtrapolateGroundFlag = true;
   double height = 0.0;
   //---
   // If ground point supplied has NaN components, return now with a NaN point.
   //---
   if ( (ossim::isnan(gpt.lat)) || (ossim::isnan(gpt.lon)) )
//       (gpt.hgt==OSSIM_DBL_NAN))
   {
      theExtrapolateGroundFlag = false;
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::extrapolate: returning..." << std::endl;
      return ossimDpt(ossim::nan(), ossim::nan());
   }
   if(ossim::isnan(gpt.hgt) == false)
   {
      height = gpt.hgt;
   }
   
   if(theSeedFunction.valid())
   {
      ossimDpt ipt;

      theSeedFunction->worldToLineSample(gpt, ipt);

      theExtrapolateGroundFlag = false;
     return ipt;
   }
   //***
   // Determine which edge is intersected by the radial, and establish
   // intersection:
   //***
   ossimDpt edgePt (gpt);
   ossimDpt image_center (theRefGndPt);
   theBoundGndPolygon.clipLineSegment(image_center, edgePt);

   //---
   // Compute an epsilon perturbation in the direction away from edgePt for
   // later computing directional derivative:
   //---
   const double  DEG_PER_MTR =  8.983152841e-06; // Equator WGS-84...
   double epsilon = theMeanGSD*DEG_PER_MTR; //degrees (latitude) per pixel
   ossimDpt deltaPt (edgePt-image_center);
   ossimDpt epsilonPt (deltaPt*epsilon/deltaPt.length());
   edgePt -= epsilonPt;
   ossimDpt edgePt_prime (edgePt - epsilonPt);
       
   //***
   // Establish image point corresponding to edge point and edgePt+epsilon:
   //***
      ossimGpt edgeGP       (edgePt.lat,       edgePt.lon,       height);//gpt.hgt);
      ossimGpt edgeGP_prime (edgePt_prime.lat, edgePt_prime.lon, height);//gpt.hgt);

   worldToLineSample(edgeGP, edgePt);
   worldToLineSample(edgeGP_prime, edgePt_prime);

   //***
   // Compute approximate directional derivatives of line and sample along
   // radial at the edge:
   //***
   double dsamp_drad = (edgePt.samp - edgePt_prime.samp)/epsilon;
   double dline_drad = (edgePt.line - edgePt_prime.line)/epsilon;

   //***
   // Now extrapolate to point of interest:
   //***
   double delta = (ossimDpt(gpt) - ossimDpt(edgeGP)).length();

   
   ossimDpt extrapolated_ip (edgePt.samp + delta*dsamp_drad,
                             edgePt.line + delta*dline_drad);

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::extrapolate: returning..." << std::endl;

   theExtrapolateGroundFlag = false;
   return extrapolated_ip;
}


//*****************************************************************************
//  METHOD: ossimSensorModel::imagingRay()
//  
//  Default implementation for computing imaging ray from image point.
//  
//*****************************************************************************
void ossimSensorModel::imagingRay(const ossimDpt& image_point,
                                  ossimEcefRay&   image_ray) const
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::imagingRay: entering..." << std::endl;

   ossimGpt start;
   ossimGpt end;

   lineSampleHeightToWorld(image_point, RAY_ORIGIN_HEIGHT, start);
   lineSampleHeightToWorld(image_point, 0.0, end);

   image_ray = ossimEcefRay(start, end);

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::imagingRay: returning..." << std::endl;
   return;
}

//*****************************************************************************
//  METHOD: ossimSensorModel::getObsCovMat()
//  
//  Default implementation for forming observation covariance matrix.
//
//    Note: At this base class level, the only error source currently
//          considered is mensuration error.  This is obviously optimistic,
//          but is included as a placeholder/example, and is presently
//          the trivial case.
//  
//*****************************************************************************
ossimSensorModel::CovMatStatus ossimSensorModel::getObsCovMat(
   const ossimDpt& /* ipos */ , NEWMAT::SymmetricMatrix& Cov, const ossim_float64 defPointingSigma)
{
   // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   // Mensuration error contribution
   // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //  Sensitivity matrix
   NEWMAT::SymmetricMatrix B(2);
   B = 0.0;
   B(1,1) = 1.0;
   B(2,2) = B(1,1);

   //  Pointing covariance matrix
   NEWMAT::SymmetricMatrix P(2);
   P = 0.0;
   P(1,1) = defPointingSigma*defPointingSigma;
   P(2,2) = P(1,1);

   //  Propagate
   NEWMAT::SymmetricMatrix Cm;
   Cm << B * P * B.t();

   // ~~~~~~~~~~~~~~~~~~~~
   // Sum total covariance
   // ~~~~~~~~~~~~~~~~~~~~
   NEWMAT::SymmetricMatrix Ctot = Cm; //+ other contributors as identified

   // ~~~~~~~~~~~~~~~~~~
   // Propagate to image
   // ~~~~~~~~~~~~~~~~~~
   NEWMAT::SymmetricMatrix Bi(2);
   Bi = 0.0;
   Bi(1,1) = 1.0;
   Bi(2,2) = Bi(1,1);

   Cov << Bi * Ctot * Bi.t();

   return ossimSensorModel::COV_PARTIAL;
}

void ossimSensorModel::computeGsd()
{
   static const char MODULE[] = "ossimSensorModel::computeGsd";

   if (theImageSize.hasNans())
   {
      std::string e = MODULE;
      e += "Error image size has nans!";
      throw ossimException(e);
   }

   //---
   // Compute gsd in the x direction from left to right points across the middle of image.
   // Compute gsd in the y direction from top to bottom points across the middle of image.
   //---

   ossim_float64 midLine = 0.0;
   ossim_float64 midSamp = 0.0;
   ossim_float64 endLine = 1.0;
   ossim_float64 endSamp = 1.0;
   if (theImageSize.x > 2)
   {
      midSamp = (theImageSize.x-1)/2.0;
      endSamp = theImageSize.x-1;
         
   }
   if (theImageSize.y > 2)
   {
      midLine = (theImageSize.y-1)/2.0;
      endLine = theImageSize.y-1;
   }
   
   ossimDpt leftDpt  (0.0,     midLine);
   ossimDpt rightDpt (endSamp, midLine);
   ossimDpt topDpt   (midSamp, 0.0);
   ossimDpt bottomDpt(midSamp, endLine);
   
   ossimGpt leftGpt;
   ossimGpt rightGpt;
   ossimGpt topGpt;
   ossimGpt bottomGpt;

   //---
   // Left point.
   // For the first point use lineSampleToWorld to get the height.
   //---
   lineSampleToWorld(leftDpt, leftGpt);
   if (leftGpt.hasNans())
   {
      std::string e = MODULE;
      e += "Error leftGpt has nans!";
      throw ossimException(e);
   }

   //---
   // Right point:
   // Use lineSampleHeightToWorld using the left height since we want the horizontal distance.
   //---
   lineSampleHeightToWorld(rightDpt, leftGpt.hgt, rightGpt);
   if (rightGpt.hasNans())
   {
      std::string e = MODULE;
      e += "Error rightGpt has nans!";
      throw ossimException(e);
   }

   //---
   // Top point:
   // Use lineSampleHeightToWorld using the left height since we want the horizontal distance.
   //---
   lineSampleHeightToWorld(topDpt, leftGpt.hgt, topGpt);
   if (topGpt.hasNans())
   {
      std::string e = MODULE;
      e += "Error topGpt has nans!";
      throw ossimException(e);
   }
   
   //---
   // Bottom point:
   // Use lineSampleHeightToWorld using the left height since we want the horizontal distance.
   //---
   lineSampleHeightToWorld(bottomDpt, leftGpt.hgt, bottomGpt);
   if (bottomGpt.hasNans())
   {
      std::string e = MODULE;
      e += "Error bottomGpt has nans!";
      throw ossimException(e);
   }

#if 0 /* Please leave for debug. (drb) */
   ossimNotify(ossimNotifyLevel_DEBUG)
      << "image size:    " << theImageSize
      << "\nleftDpt:   " << leftDpt
      << "\nrightDpt:  " << rightDpt
      << "\ntopDpt:    " << topDpt
      << "\nbottomDpt: " << bottomDpt      
      << "\nleftGpt:   " << leftGpt
      << "\nrightGpt:  " << rightGpt
      << "\ntopGpt:    " << topGpt
      << "\nbottomGpt: " << bottomGpt      
      << "\n";
#endif
      
   theGSD.x   = leftGpt.distanceTo(rightGpt)/(rightDpt.x-leftDpt.x);
   theGSD.y   = topGpt.distanceTo(bottomGpt)/(bottomDpt.y-topDpt.y);
   theMeanGSD = (theGSD.x + theGSD.y)/2.0;

   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "ossimSensorModel::computGsd DEBUG:"
         << "\ntheGSD:     " << theGSD
         << "\ntheMeanGSD: " << theMeanGSD << std::endl;
   }
}

//*****************************************************************************
//  STATIC METHOD: ossimSensorModel::writeGeomTemplate
//  
//  Outputs a sample geometry KWL to stream provided.
//  
//*****************************************************************************
void ossimSensorModel::writeGeomTemplate(ostream& os)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::writeGeomTemplate:entering..." << std::endl;
   
   os << "//***\n"
      << "// Base-class ossimSensorModel Keywords:\n"
      << "//***\n"
      << ossimKeywordNames::ID_KW << ":  <string>\n"
      << SENSOR_ID_KW << ": <string>\n"
      << ossimKeywordNames::NUMBER_LINES_KW << ": <int>\n"
      << ossimKeywordNames::NUMBER_SAMPLES_KW << ": <int>\n"
      << REF_GPT_LAT_KW << ": <decimal degrees>\n"
      << REF_GPT_LON_KW << ": <decimal degrees>\n"
      << REF_GPT_HGT_KW << ": <float meters>\n"
      << REF_IPT_LINE_KW << ": <float>\n"
      << REF_IPT_SAMP_KW << ": <float>\n"
      << ossimKeywordNames::METERS_PER_PIXEL_Y_KW << ": <float meters>\n"
      << ossimKeywordNames::METERS_PER_PIXEL_X_KW << ": <float meters>\n"
      << ossimKeywordNames::UL_LAT_KW << ": <decimal degrees>\n"
      << ossimKeywordNames::UL_LON_KW << ": <decimal degrees>\n"
      << ossimKeywordNames::UR_LAT_KW << ": <decimal degrees>\n"
      << ossimKeywordNames::UR_LON_KW << ": <decimal degrees>\n"
      << ossimKeywordNames::LR_LAT_KW << ": <decimal degrees>\n"
      << ossimKeywordNames::LR_LON_KW << ": <decimal degrees>\n"
      << ossimKeywordNames::LL_LAT_KW << ": <decimal degrees>\n"
      << ossimKeywordNames::LL_LON_KW << ": <decimal degrees>\n"
      << "\n"
      << "//***\n"
      << "// Repeat following four entries for each adjustable parameter:\n"
      << "//***\n"
//       << PARAM_PREFIX << "N." << PARAM_NAME_KW << ": <string>\n" 
//       << PARAM_PREFIX << "N." << PARAM_UNITS_KW << ": <string>\n" 
//       << PARAM_PREFIX << "N." << PARAM_VALUE_KW << ": <float>\n" 
//       << PARAM_PREFIX << "N." << PARAM_SIGMA_KW << ": <float>\n"
      << std::endl;

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimSensorModel::writeGeomTemplate: returning..." << std::endl;
   return;
}

ossim_uint32 
ossimSensorModel::degreesOfFreedom()const
{
   ossim_uint32 dof = 0;
   ossim_uint32 idx = 0;
   ossim_uint32 numAdj = getNumberOfAdjustableParameters();
   for(idx = 0; idx < numAdj; ++idx)
   {
      if(!isParameterLocked(idx))
      {
         ++dof;
      }
   }
   
   return dof;
}


// build a error equation according to a gcp
void ossimSensorModel::buildErrorEquation ( const ossimTieGpt& tiePoint, int nType, NEWMAT::Matrix &A,
										   NEWMAT::Matrix &B, NEWMAT::ColumnVector &L, double pstep_scale)
{
	int np = getNumberOfAdjustableParameters();
	int no = 2; //image observation

	A.ReSize(no,np);
	B.ReSize(no,2);
	L.ReSize(no);
	//Zeroify matrices that will be accumulated
	A = 0.0;
	B = 0.0;
	L = 0.0;	

	ossimDpt imDerp;

	ossimDpt resIm;
	resIm = tiePoint.tie - forward(tiePoint);
	L(1) = resIm.x;
	L(2) = resIm.y;
	//compute all image derivatives regarding parametres for the tie point position
	for(int p=0;p<np;++p)
	{
		imDerp = getForwardDeriv( p , tiePoint , pstep_scale);
		A.element(0, p) = imDerp.x;
		A.element(1, p) = imDerp.y;
	}

	if(0 == nType)
	{// if the unknown corresponding points
		for(int p=0;p<2;++p)
		{
			imDerp = getCoordinateForwardDeriv( p , tiePoint , pstep_scale);
			B.element(0, p) = imDerp.x;
			B.element(1, p) = imDerp.y;
		}
	}
}

double
	ossimSensorModel::optimizeFit(const ossimTieGptSet& tieSet, std::vector<int> optimizerList, double* /* targetVariance */)
{
	//use a simple Levenberg-Marquardt non-linear optimization
	//note : please limit the number of tie points
	//
	//INPUTS: requires Jacobian matrix (partial derivatives with regards to parameters)
	//OUPUTS: will also compute parameter covariance matrix
	//
	//TBD: use targetVariance!

	int np = (int)optimizerList.size();
	int nobs = tieSet.size();

	//setup initail values
	int iter=0;
	int iter_max = 200;
	double minResidue = 1e-10; //TBC
	double minDelta = 1e-10; //TBC

	//build Least Squares initial normal equation
	// don't waste memory, add samples one at a time
	NEWMAT::SymmetricMatrix A;
	NEWMAT::ColumnVector residue;
	NEWMAT::ColumnVector projResidue;
	double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1
	buildNormalEquation(tieSet, optimizerList, A, residue, projResidue, deltap_scale);
	double ki2=residue.SumSquare();

	//get current adjustment (between -1 and 1 normally) and convert to ColumnVector
	ossimAdjustmentInfo cadj;
	getAdjustment(cadj);
	std::vector< ossimAdjustableParameterInfo >& parmlist = cadj.getParameterList();
	NEWMAT::ColumnVector cparm(np), nparm(np);
	for(int n=0;n<np;++n)
	{
		cparm(n+1) = parmlist[optimizerList[n]].getParameter();
	}

	double damping_speed = 2.0;
	//find max diag element for A
	double maxdiag=0.0;
	for(int d=1;d<=np;++d) {
		if (maxdiag < A(d,d)) maxdiag=A(d,d);
	}
	double damping = 1e-3 * maxdiag;
	double olddamping = 0.0;
	bool found = false;

	//DEBUG TBR
	// cout<<"rms="<<sqrt(ki2/nobs)<<" ";
	// cout.flush();

	while ( (!found) && (iter < iter_max) ) //non linear optimization loop
	{
		bool decrease = false;

		do
		{
			//add damping update to normal matrix
			for(int d=1;d<=np;++d) A(d,d) += damping - olddamping;
			olddamping = damping;

			NEWMAT::ColumnVector deltap = solveLeastSquares(A, projResidue);
			//int nCols = A.Ncols();
			//int nRows = A.Nrows();
			//arma::mat matX(nRows, nCols);
			//arma::vec matY(nRows);

			//for (int i = 0;i < nRows;++i)
			//{
			//	matY[i] = projResidue(i+1);
			//	for (int j = 0;j < nCols;++j)
			//	{
			//		matX(i,j) = A(i+1,j+1);
			//	}
			//}
			//cout<<matX;
			//cout<<matY;
			//double lambda1 = 1e-6;
			//double lambda2 = 0;
			//LARS lars(false, lambda1, lambda2);
			//vec beta;
			//lars.Regress(matX, matY.unsafe_col(0), beta, false /* do not transpose */);

			//NEWMAT::ColumnVector deltap(beta.n_elem);
			//for(int i = 0;i < beta.n_elem;i++)
			//{
			//	deltap[i] = beta[i];
			//}

			if (deltap.NormFrobenius() <= minDelta) 
			{
				found = true;
			} else {
				//update adjustment
				nparm = cparm + deltap;
				for(int n=0;n<np;++n)
				{
					setAdjustableParameter(optimizerList[n], nparm(n+1), false); //do not update now, wait
				}
				updateModel();

				//check residue is reduced
				NEWMAT::ColumnVector newresidue = getResidue(tieSet);
				double newki2=newresidue.SumSquare();
				double res_reduction = (ki2 - newki2) / (deltap.t()*(deltap*damping + projResidue)).AsScalar();
				//DEBUG TBR
				cout<<sqrt(newki2/nobs)<<" ";
				cout.flush();

				if (res_reduction > 0)
				{
					//accept new parms
					cparm = nparm;
					ki2=newki2;

					deltap_scale = max(1e-15, deltap.NormInfinity()*1e-4);

					buildNormalEquation(tieSet, optimizerList, A, residue, projResidue, deltap_scale);
					olddamping = 0.0;

					found = ( projResidue.NormInfinity() <= minResidue );
					//update damping factor
					damping *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
					damping_speed = 2.0;
					decrease = true;
				} else {
					//cancel parameter update
					for(int n=0;n<np;++n)
					{
						setAdjustableParameter(optimizerList[n], nparm(n+1), false); //do not update right now
					}
					updateModel();

					damping *= damping_speed;
					damping_speed *= 2.0;
				}
			}
		} while (!decrease && !found);
		++iter;
	}

	//DEBUG TBR
	cout<<endl;

	//compute parameter correlation
	// use normal matrix inverse
	//TBD

	return ki2/nobs;
}

double
ossimSensorModel::optimizeFit(const ossimTieGptSet& tieSet, double* /* targetVariance */)
{
   //use a simple Levenberg-Marquardt non-linear optimization
   //note : please limit the number of tie points
   //
   //INPUTS: requires Jacobian matrix (partial derivatives with regards to parameters)
   //OUPUTS: will also compute parameter covariance matrix
   //
   //TBD: use targetVariance!
 
   int np = getNumberOfAdjustableParameters();
   int nobs = tieSet.size();

   //setup initail values
   int iter=0;
   int iter_max = 200;
   double minResidue = 1e-10; //TBC
   double minDelta = 1e-10; //TBC

   //build Least Squares initial normal equation
   // don't waste memory, add samples one at a time
   NEWMAT::SymmetricMatrix A;
   NEWMAT::ColumnVector residue;
   NEWMAT::ColumnVector projResidue;
   double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1
   buildNormalEquation(tieSet, A, residue, projResidue, deltap_scale);
   double ki2=residue.SumSquare();

   //get current adjustment (between -1 and 1 normally) and convert to ColumnVector
   ossimAdjustmentInfo cadj;
   getAdjustment(cadj);
   std::vector< ossimAdjustableParameterInfo >& parmlist = cadj.getParameterList();
   NEWMAT::ColumnVector cparm(np), nparm(np);
   for(int n=0;n<np;++n)
   {
      cparm(n+1) = parmlist[n].getParameter();
   }

   double damping_speed = 2.0;
   //find max diag element for A
   double maxdiag=0.0;
   for(int d=1;d<=np;++d) {
      if (maxdiag < A(d,d)) maxdiag=A(d,d);
   }
   double damping = 1e-3 * maxdiag;
   double olddamping = 0.0;
   bool found = false;

   //DEBUG TBR
   // cout<<"rms="<<sqrt(ki2/nobs)<<" ";
   // cout.flush();

   while ( (!found) && (iter < iter_max) ) //non linear optimization loop
   {
      bool decrease = false;

      do
      {
         //add damping update to normal matrix
         for(int d=1;d<=np;++d) A(d,d) += damping - olddamping;
         olddamping = damping;

         NEWMAT::ColumnVector deltap = solveLeastSquares(A, projResidue);

         if (deltap.NormFrobenius() <= minDelta) 
         {
            found = true;
         } else {
            //update adjustment
            nparm = cparm + deltap;
            for(int n=0;n<np;++n)
            {
               setAdjustableParameter(n, nparm(n+1), false); //do not update now, wait
            }
            updateModel();

            //check residue is reduced
            NEWMAT::ColumnVector newresidue = getResidue(tieSet);
            double newki2=newresidue.SumSquare();
            double res_reduction = (ki2 - newki2) / (deltap.t()*(deltap*damping + projResidue)).AsScalar();
 //DEBUG TBR
       cout<<sqrt(newki2/nobs)<<" ";
       cout.flush();

            if (res_reduction > 0)
            {
               //accept new parms
               cparm = nparm;
               ki2=newki2;

               deltap_scale = max(1e-15, deltap.NormInfinity()*1e-4);

               buildNormalEquation(tieSet, A, residue, projResidue, deltap_scale);
               olddamping = 0.0;

               found = ( projResidue.NormInfinity() <= minResidue );
               //update damping factor
               damping *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
               damping_speed = 2.0;
               decrease = true;
            } else {
               //cancel parameter update
               for(int n=0;n<np;++n)
               {
                  setAdjustableParameter(n, nparm(n+1), false); //do not update right now
               }
               updateModel();

               damping *= damping_speed;
               damping_speed *= 2.0;
            }
         }
      } while (!decrease && !found);
      ++iter;
   }

//DEBUG TBR
cout<<endl;

   //compute parameter correlation
   // use normal matrix inverse
   //TBD

   return ki2/nobs;
}

double ossimSensorModel::optimizeFit(const vector< ossimTieFeature >& tieFeatureList, double* targetVariance/* = 0*/)
{
	return eigen_levmar_optimization(tieFeatureList, targetVariance);

	//use a simple Levenberg-Marquardt non-linear optimization
	//note : please limit the number of tie points
	//
	//INPUTS: requires Jacobian matrix (partial derivatives with regards to parameters)
	//OUPUTS: will also compute parameter covariance matrix
	//
	//TBD: use targetVariance!
	int np = getNumberOfAdjustableParameters();
	int nobs = tieFeatureList.size();
	//setup initail values
	int iter=0;
	int iter_max = 200;    //ww1130
	double minResidue = 1e-10; //TBC
	double minDelta = 1e-10; //TBC
	//build Least Squares initial normal equation
	// don't waste memory, add samples one at a time
	NEWMAT::SymmetricMatrix A;
	NEWMAT::ColumnVector residue;
	NEWMAT::ColumnVector projResidue;
	double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1
	bool useImageObs = true;	//2010.1.18 loong
	buildNormalEquation(tieFeatureList, A, residue, projResidue, deltap_scale, useImageObs);	//2010.1.18 loong
	double ki2=residue.SumSquare();
	//get current adjustment (between -1 and 1 normally) and convert to ColumnVector
	ossimAdjustmentInfo cadj;
	getAdjustment(cadj);
	std::vector< ossimAdjustableParameterInfo >& parmlist = cadj.getParameterList();
	NEWMAT::ColumnVector cparm(np), nparm(np);
	for(int n=0;n<np;++n)
	{
		cparm(n+1) = parmlist[n].getParameter();
	}
	double damping_speed = 2.0;
	//find max diag element for A
	double maxdiag=0.0;
	for(int d=1;d<=np;++d) {
		if (maxdiag < A(d,d)) maxdiag=A(d,d);
	}
	double damping = 1e-3 * maxdiag;
	double olddamping = 0.0;
	bool found = false;
	//DEBUG TBR
	// cout<<"rms="<<sqrt(ki2/nobs)<<" ";
	// cout.flush();
	while ( (!found) && (iter < iter_max) ) //non linear optimization loop
	{
		bool decrease = false;
		do
		{
			//add damping update to normal matrix
			for(int d=1;d<=np;++d) A(d,d) += damping - olddamping;
			olddamping = damping;

			NEWMAT::ColumnVector deltap = solveLeastSquares(A, projResidue);
			//cout<<deltap<<endl; //2010.1.18 loong
			if (deltap.NormFrobenius() <= minDelta) 
			{
				found = true;
			} else {
				//update adjustment
				nparm = cparm + deltap;
				for(int n=0;n<np;++n)
				{
					setAdjustableParameter(n, nparm(n+1), false); //do not update now, wait
				}
				updateModel();
				//ossimKeywordlist ge;
				//saveState(ge);
				//cout<<ge<<endl<<endl;
				//check residue is reduced
				NEWMAT::ColumnVector newresidue = getResidue(tieFeatureList, useImageObs);	//2010.1.18 loong
				double newki2=newresidue.SumSquare();
				double res_reduction = (ki2 - newki2) / (deltap.t()*(deltap*damping + projResidue)).AsScalar();
				//DEBUG TBR
				cout<<sqrt(newki2/nobs)<<" ";
				cout.flush();
				if (res_reduction > 0)
				{
					//accept new parms
					cparm = nparm;
					ki2=newki2;
					deltap_scale = max(1e-15, deltap.NormInfinity()*1e-4);
					buildNormalEquation(tieFeatureList, A, residue, projResidue, deltap_scale, useImageObs);	//2010.1.18 loong
					olddamping = 0.0;
					found = ( projResidue.NormInfinity() <= minResidue );
					//update damping factor
					damping *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
					damping_speed = 2.0;
					decrease = true;
				} else {
					//cancel parameter update
					for(int n=0;n<np;++n)
					{
						setAdjustableParameter(n, nparm(n+1), false); //do not update right now
					}
					updateModel();
					damping *= damping_speed;
					damping_speed *= 2.0;
				}
			}
		} while (!decrease && !found);
		++iter;
	}
	//DEBUG TBR
	cout<<endl;
	//compute parameter correlation
	// use normal matrix inverse
	//TBD
	return ki2/nobs;
}

double ossimSensorModel::eigen_levmar_optimization(const vector< ossimTieFeature >& tieFeatureList, double* targetVariance/* = 0*/)
{
	int np = getNumberOfAdjustableParameters();
	int nobs = tieFeatureList.size();

	int info;
	//double fnorm, covfac;
	Eigen::VectorXd x(np);

	//get current adjustment (between -1 and 1 normally) and convert to ColumnVector
	ossimAdjustmentInfo cadj;
	getAdjustment(cadj);
	std::vector< ossimAdjustableParameterInfo >& parmlist = cadj.getParameterList();
	for(int n=0;n<np;++n)
	{
		x[n] = parmlist[n].getParameter();
	}


	// do the computation
	lmder_functor functor(np, 2*nobs);
	optimizeStruct optStruct;
	optStruct.tieFeatureList = tieFeatureList;
	optStruct.pThis = this;
	functor.pData = &optStruct;
	Eigen::LevenbergMarquardt<lmder_functor> lm(functor);
	info = lm.minimize(x);

	for(int n=0;n<np;++n)
	{
		setAdjustableParameter(n, x[n], false); //do not update right now
		cout<<x[n]<<endl;
	}
	updateModel();
	return 0.0;
}

void
ossimSensorModel::buildNormalEquation(const ossimTieGptSet& tieSet,
                                      NEWMAT::SymmetricMatrix& A,
                                      NEWMAT::ColumnVector& residue,
                                      NEWMAT::ColumnVector& projResidue,
                                      double pstep_scale)
{
   //goal:       build Least Squares system
   //constraint: never store full Jacobian matrix in memory (can be huge)
   //            so we build the matrices incrementally
   // the system can be built using forward() or inverse() depending on the projection capabilities : useForward()
   //
   //TBD : add covariance matrix for each tie point

   //init
   int np = getNumberOfAdjustableParameters();
   int dimObs;
   bool useImageObs = useForward(); //caching
   if (useImageObs)
   {
      dimObs = 2; //image observation
   } else {
      dimObs = 3; //ground observations
   }
   int no = dimObs * tieSet.size(); //number of observations

   A.ReSize(np);
   residue.ReSize(no);
   projResidue.ReSize(np);
   //Zeroify matrices that will be accumulated
   A           = 0.0;
   projResidue = 0.0;

   const vector<ossimRefPtr<ossimTieGpt> >& theTPV = tieSet.getTiePoints();
   vector<ossimRefPtr<ossimTieGpt> >::const_iterator tit;
   unsigned long c=1;

   if (useImageObs)
   { 
     //image observations 
      std::vector<ossimDpt> imDerp(np);
     ossimDpt resIm;
     // loop on tie points
      for (tit = theTPV.begin() ; tit != theTPV.end() ; ++tit)
      {
         //compute residue
         resIm = (*tit)->tie - forward(*(*tit));
         residue(c++) = resIm.x;
         residue(c++) = resIm.y;

         //compute all image derivatives regarding parametres for the tie point position
         for(int p=0;p<np;++p)
         {
            imDerp[p] = getForwardDeriv( p , *(*tit) , pstep_scale);
         }

         //compute influence of tie point on all sytem elements
         for(int p1=0;p1<np;++p1)
         {        
            //proj residue: J * residue
            projResidue.element(p1) += imDerp[p1].x * resIm.x + imDerp[p1].y * resIm.y;

            //normal matrix A = transpose(J)*J
            for(int p2=p1;p2<np;++p2)
            {
               A.element(p1,p2) += imDerp[p1].x * imDerp[p2].x + imDerp[p1].y * imDerp[p2].y;
            }
         }
      }
   }
   else
   {
      // ground observations
      std::vector<ossimGpt>  gdDerp(np);
	  ossimGpt gd, resGd;
	  ossimDpt dptres,dptgd;
      // loop on tie points
	  if(!m_proj) return;	// ww
      for (tit = theTPV.begin() ; tit != theTPV.end() ; ++tit)
      {
         ////compute residue
         //gd = inverse((*tit)->tie);
         //residue(c++) = resGd.lon = ((*tit)->lon - gd.lon) * 100000.0;
         //residue(c++) = resGd.lat = ((*tit)->lat - gd.lat) * 100000.0 * cos(gd.lat / 180.0 * M_PI);
         //residue(c++) = resGd.hgt = (*tit)->hgt - gd.hgt; //TBD : normalize to meters?

		  gd =inverse((*tit)->tie);
		  dptres=m_proj->forward((*tit)->getGroundPoint());
		  dptgd=m_proj->forward(gd);
		  residue(c++)= resGd.lon = dptres.lon - dptgd.lon; //TBC : approx meters
		  residue(c++)= resGd.lat = dptres.lat - dptgd.lat;// * 100000.0 * cos(gd.lat / 180.0 * M_PI);
		  residue(c++)=  resGd.hgt = (*tit)->hgt - gd.hgt;

         //compute all image derivatives regarding parametres for the tie point position
         for(int p=0;p<np;++p)
         {
            gdDerp[p] = getInverseDeriv( p , (*tit)->tie, pstep_scale);
         }

         //compute influence of tie point on all sytem elements
         for(int p1=0;p1<np;++p1)
         {        
            //proj residue: J * residue
            projResidue.element(p1) += gdDerp[p1].lon * resGd.lon + gdDerp[p1].lat * resGd.lat + gdDerp[p1].hgt * resGd.hgt; //TBC

            //normal matrix A = transpose(J)*J
            for(int p2=p1;p2<np;++p2)
            {
               A.element(p1,p2) += gdDerp[p1].lon * gdDerp[p2].lon + gdDerp[p1].lat * gdDerp[p2].lat + gdDerp[p1].hgt * gdDerp[p2].hgt;
            }
         }
      }

   } //end of if (useImageObs)
}

void
	ossimSensorModel::buildNormalEquation(const ossimTieGptSet& tieSet,
	std::vector<int> optimizerList,
	NEWMAT::SymmetricMatrix& A,
	NEWMAT::ColumnVector& residue,
	NEWMAT::ColumnVector& projResidue,
	double pstep_scale)
{
	//goal:       build Least Squares system
	//constraint: never store full Jacobian matrix in memory (can be huge)
	//            so we build the matrices incrementally
	// the system can be built using forward() or inverse() depending on the projection capabilities : useForward()
	//
	//TBD : add covariance matrix for each tie point

	//init
	int np = (int)optimizerList.size();
	int dimObs;
	bool useImageObs = useForward(); //caching
	if (useImageObs)
	{
		dimObs = 2; //image observation
	} else {
		dimObs = 3; //ground observations
	}
	int no = dimObs * tieSet.size(); //number of observations

	A.ReSize(np);
	residue.ReSize(no);
	projResidue.ReSize(np);
	//Zeroify matrices that will be accumulated
	A           = 0.0;
	projResidue = 0.0;

	const vector<ossimRefPtr<ossimTieGpt> >& theTPV = tieSet.getTiePoints();
	vector<ossimRefPtr<ossimTieGpt> >::const_iterator tit;
	unsigned long c=1;

	if (useImageObs)
	{ 
		//image observations 
		std::vector<ossimDpt> imDerp(np);
		ossimDpt resIm;
		// loop on tie points
		//#pragma omp parallel for
		//for (int ii = 0;ii < (int)tieSet.getTiePoints().size();++ii)
		for (tit = theTPV.begin() ; tit != theTPV.end() ; ++tit)
		{
			//tit = theTPV.begin()+ii;
			//compute residue
			resIm = (*tit)->tie - forward(*(*tit));
			residue(c++) = resIm.x;
			residue(c++) = resIm.y;

			//compute all image derivatives regarding parametres for the tie point position
			for(int p=0;p<np;++p)
			{
				imDerp[p] = getForwardDeriv( optimizerList[p] , *(*tit) , pstep_scale);
			}

			//compute influence of tie point on all sytem elements
			for(int p1=0;p1<np;++p1)
			{        
				//proj residue: J * residue
				projResidue.element(p1) += imDerp[p1].x * resIm.x + imDerp[p1].y * resIm.y;

				//normal matrix A = transpose(J)*J
				for(int p2=p1;p2<np;++p2)
				{
					A.element(p1,p2) += imDerp[p1].x * imDerp[p2].x + imDerp[p1].y * imDerp[p2].y;
				}
			}
		}
	}
	else
	{
		// ground observations
		std::vector<ossimGpt>  gdDerp(np);
		ossimGpt gd, resGd;
		// loop on tie points
		if(!m_proj) return;	// ww
		for (tit = theTPV.begin() ; tit != theTPV.end() ; ++tit)
		{
			//compute residue
			gd = inverse((*tit)->tie);
			residue(c++) = resGd.lon = ((*tit)->lon - gd.lon) * 100000.0;
			residue(c++) = resGd.lat = ((*tit)->lat - gd.lat) * 100000.0 * cos(gd.lat / 180.0 * M_PI);
			residue(c++) = resGd.hgt = (*tit)->hgt - gd.hgt; //TBD : normalize to meters?

			//compute all image derivatives regarding parametres for the tie point position
			for(int p=0;p<np;++p)
			{
				gdDerp[p] = getInverseDeriv( optimizerList[p] , (*tit)->tie, pstep_scale);
			}

			//compute influence of tie point on all sytem elements
			for(int p1=0;p1<np;++p1)
			{        
				//proj residue: J * residue
				projResidue.element(p1) += gdDerp[p1].lon * resGd.lon + gdDerp[p1].lat * resGd.lat + gdDerp[p1].hgt * resGd.hgt; //TBC

				//normal matrix A = transpose(J)*J
				for(int p2=p1;p2<np;++p2)
				{
					A.element(p1,p2) += gdDerp[p1].lon * gdDerp[p2].lon + gdDerp[p1].lat * gdDerp[p2].lat + gdDerp[p1].hgt * gdDerp[p2].hgt;
				}
			}
		}

	} //end of if (useImageObs)
}

void
ossimSensorModel::buildNormalEquation(const vector< ossimTieFeature >& tieFeatureList,
									 NEWMAT::SymmetricMatrix& A,
									 NEWMAT::ColumnVector& residue,
									 NEWMAT::ColumnVector& projResidue,
									 double pstep_scale, 
									 bool useImageObs /* = false // 2010.1.18 loong*/)
{
	//goal:       build Least Squares system
	//constraint: never store full Jacobian matrix in memory (can be huge)
	//            so we build the matrices incrementally
	// the system can be built using forward() or inverse() depending on the projection capabilities : useForward()
	//
	//TBD : add covariance matrix for each tie point
	//init
	int np = getNumberOfAdjustableParameters();
	int dimObs;
	//bool useImageObs = useForward(); //caching			//2010.1.18 loong
	if (useImageObs)
	{
		dimObs = 2; //image observation
	} else {
		dimObs = 3; //ground observations
	}
	int nobs = tieFeatureList.size();
	//int no = dimObs * nobs; //number of observations
	int no = getNumofObservations(tieFeatureList, useImageObs);
	A.ReSize(np);
	residue.ReSize(no);
	projResidue.ReSize(np);
	//Zeroify matrices that will be accumulated
	A           = 0.0;
	projResidue = 0.0;
	unsigned long c = 1;
	double N=0;
	double e2=0;
	double a=6378137;
	double b=6356752.3;
	e2=(a*a-b*b)/(a*a) ;
	int index=0;
	int i;
	if (useImageObs)
	{ 
		std::vector<ossimDpt> imDerp(np);
		ossimDpt resIm;
		for (i = 0 ; i < nobs ; ++i)
		{
			switch(tieFeatureList[i].getTieFeatureType())
			{
			case ossimTieFeature::ossimTieFeatureType::ossimTiePointPoint:
				{
					resIm = tieFeatureList[i].getImageFeature().m_Points[0] - forward(tieFeatureList[i].getGroundFeature().m_Points[0]);
					residue(c++) = resIm.x;
					residue(c++) = resIm.y;
					//compute all image derivatives regarding parametres for the tie point position
					for(int p=0;p<np;++p)
					{
						imDerp[p] = getForwardDeriv( p , tieFeatureList[i].getGroundFeature().m_Points[0], pstep_scale);
					}
					//compute influence of tie point on all sytem elements
					for(int p1=0;p1<np;++p1)
					{        
						//proj residue: J * residue
						projResidue.element(p1) += imDerp[p1].x * resIm.x + imDerp[p1].y * resIm.y;
						//normal matrix A = transpose(J)*J
						for(int p2=p1;p2<np;++p2)
						{
							A.element(p1,p2) += imDerp[p1].x * imDerp[p2].x + imDerp[p1].y * imDerp[p2].y;
						}
					}
					break;
				}
			case ossimTieFeature::ossimTieFeatureType::ossimTieLineLine:
				{
					ossimDpt newDpt1 = forward(tieFeatureList[i].getGroundFeature().m_Points[0]);
					ossimDpt newDpt2 = forward(tieFeatureList[i].getGroundFeature().m_Points[1]);
					ossimDLine newDLine(newDpt1, newDpt2);
					ossimDLine oldDLine(tieFeatureList[i].getImageFeature().m_Points[0], tieFeatureList[i].getImageFeature().m_Points[1]);
					ossimDpt res = oldDLine.DistanceAndSegment(newDLine);

					residue(c++) = res.x;
					residue(c++) = res.y;

					double pstep_scale = 1e-4;
					static double den = 0.5 / pstep_scale;
					for(int p=0;p<np;++p)
					{
						double middle = getAdjustableParameter(p);

						setAdjustableParameter(p, middle + pstep_scale, true);

						ossimDpt newDpt1_ = forward(tieFeatureList[i].getGroundFeature().m_Points[0]);
						ossimDpt newDpt2_ = forward(tieFeatureList[i].getGroundFeature().m_Points[1]);
						ossimDLine newDLine_(newDpt1_, newDpt2_);
						ossimDpt dist1 = oldDLine.DistanceAndSegment(newDLine_);

						setAdjustableParameter(p, middle - pstep_scale, true);
						newDpt1_ = forward(tieFeatureList[i].getGroundFeature().m_Points[0]);
						newDpt2_ = forward(tieFeatureList[i].getGroundFeature().m_Points[1]);
						newDLine_.setPoints(newDpt1_, newDpt2_);
						ossimDpt dist2 = oldDLine.DistanceAndSegment(newDLine_);

						setAdjustableParameter(p, middle, true);
						double derivative_x = (dist1.x - dist2.x) * den;
						double derivative_y = (dist1.y - dist2.y) * den;

						imDerp[p].x = derivative_x;
						imDerp[p].y = derivative_y;
					}

					for(int p1=0;p1<np;++p1)
					{        
						//proj residue: J * residue
						projResidue.element(p1) += imDerp[p1].x * residue(c - 2);
						projResidue.element(p1) += imDerp[p1].y * residue(c - 1);
						//normal matrix A = transpose(J)*J
						for(int p2=p1;p2<np;++p2)
						{
							A.element(p1,p2) +=  imDerp[p1].x * imDerp[p2].x + imDerp[p1].y * imDerp[p2].y;}
					}
					
					//ossimDpt newDpt1 = forward(tieFeatureList[i].getGroundFeature().m_Points[0]);
					//ossimDpt gpt1 = tieFeatureList[i].getImageFeature().m_Points[0];
					//ossimDpt gpt2 = tieFeatureList[i].getImageFeature().m_Points[1];
					//double hemline = sqrt((gpt1.x - gpt2.x)*(gpt1.x - gpt2.x) + (gpt1.y - gpt2.y)*(gpt1.y - gpt2.y));
					//double k1 = (gpt1.y - gpt2.y) / (hemline + small_quantity);
					//double k2 = (gpt1.x - gpt2.x) / (hemline + small_quantity);

					////for the first endpoint of a line
					//residue(c++) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt1.x + k2 * newDpt1.y;

					//for(int p=0;p<np;++p)
					//{
					//	imDerp[p] = getForwardDeriv( p , tieFeatureList[i].getGroundFeature().m_Points[0], pstep_scale);
					//}
					////compute influence of tie point on all system elements
					//for(int p1=0;p1<np;++p1)
					//{        
					//	//proj residue: J * residue
					//	projResidue.element(p1) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * residue(c - 1);
					//	//normal matrix A = transpose(J)*J
					//	for(int p2=p1;p2<np;++p2)
					//	{
					//		A.element(p1,p2) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * (k1 * imDerp[p2].x - k2 * imDerp[p2].y);
					//	}
					//}

					////for the second endpoint of a line

					//ossimDpt newDpt2 = forward(tieFeatureList[i].getGroundFeature().m_Points[1]);
					//residue(c++) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt2.x + k2 * newDpt2.y;
					//for(int p=0;p<np;++p)
					//{
					//	imDerp[p] = getForwardDeriv( p , tieFeatureList[i].getGroundFeature().m_Points[1], pstep_scale);
					//}
					////compute influence of tie point on all system elements
					//for(int p1=0;p1<np;++p1)
					//{        
					//	//proj residue: J * residue
					//	projResidue.element(p1) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * residue(c - 1);
					//	//normal matrix A = transpose(J)*J
					//	for(int p2=p1;p2<np;++p2)
					//	{
					//		A.element(p1,p2) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * (k1 * imDerp[p2].x - k2 * imDerp[p2].y);
					//	}
					//}
					break;
				}
			case ossimTieFeature::ossimTieFeatureType::ossimTieAreaArea:
				{
					////
					//vector<ossimDpt> newPoints;
					//for(unsigned int iVertex = 0;iVertex < tieFeatureList[i].getGroundFeature().m_Points.size();iVertex++)
					//{
					//	newPoints.push_back(forward(tieFeatureList[i].getGroundFeature().m_Points[iVertex]));
					//}
					//ossimDArea oldArea(tieFeatureList[i].getImageFeature().m_Points);
					//ossimDArea newArea(newPoints);
					//ossimDpt tmpDisVector;
					//oldArea.DistanceFromArea(newArea, &tmpDisVector);
					////ossimDpt tmpDisVector = oldArea.DistanceFromAreaMid(newArea, &nIndex);
					//residue(c++) = -tmpDisVector.x;
					//residue(c++) = -tmpDisVector.y;

					//for(int p=0;p<np;++p)
					//{
					//	imDerp[p] = getForwardDeriv( p , tieFeatureList[i].getGroundFeature().m_Points[nIndex], pstep_scale);
					//}


					//// 距离X函数对像素坐标微分
					//ossimDpt patialDistance_X;
					//// 距离Y函数对像素坐标微分
					//ossimDpt patialDistance_Y;
					//oldArea.getPointDistanceDeriv(newArea.m_Points[nIndex], &patialDistance_X, &patialDistance_Y);

					////compute influence of tie point on all system elements
					//for(int p1=0;p1<np;++p1)
					//{        
					//	//proj residue: J * residue
					//	projResidue.element(p1) += (patialDistance_X.x * imDerp[p1].x + patialDistance_X.y * imDerp[p1].y) * residue(c - 2);
					//	projResidue.element(p1) += (patialDistance_Y.x * imDerp[p1].x + patialDistance_Y.y * imDerp[p1].y) * residue(c - 1);
					//	//normal matrix A = transpose(J)*J
					//	for(int p2=p1;p2<np;++p2)
					//	{
					//		A.element(p1,p2) += (patialDistance_X.x * imDerp[p1].x + patialDistance_X.y * imDerp[p1].y) * (patialDistance_X.x * imDerp[p2].x + patialDistance_X.y * imDerp[p2].y);
					//		A.element(p1,p2) += (patialDistance_Y.x * imDerp[p1].x + patialDistance_Y.y * imDerp[p1].y) * (patialDistance_Y.x * imDerp[p2].x + patialDistance_Y.y * imDerp[p2].y);
					//	}
					//}
					break;
				}
				/*
			case ossimTieFeature::ossimFeatureType::ossimGptFeature:
				{
					resIm = tieFeatureList[i].m_TiePoints[0].getImagePoint() - forward(tieFeatureList[i].m_TiePoints[0].getGroundPoint());
					residue(c++) = resIm.x;
					residue(c++) = resIm.y;
					//compute all image derivatives regarding parametres for the tie point position
					for(int p=0;p<np;++p)
					{
						imDerp[p] = getForwardDeriv( p , tieFeatureList[i].m_TiePoints[0].getGroundPoint() , pstep_scale);
					}
					//compute influence of tie point on all sytem elements
					for(int p1=0;p1<np;++p1)
					{        
						//proj residue: J * residue
						projResidue.element(p1) += imDerp[p1].x * resIm.x + imDerp[p1].y * resIm.y;
						//normal matrix A = transpose(J)*J
						for(int p2=p1;p2<np;++p2)
						{
							A.element(p1,p2) += imDerp[p1].x * imDerp[p2].x + imDerp[p1].y * imDerp[p2].y;
						}
					}
					break;
				}
			case ossimTieFeature::ossimFeatureType::ossimLineFeature:
				{
					ossimDpt newDpt1 = forward(tieFeatureList[i].m_TiePoints[0].getGroundPoint());
					ossimDpt gpt1 = tieFeatureList[i].m_TiePoints[0].getImagePoint();
					ossimDpt gpt2 = tieFeatureList[i].m_TiePoints[1].getImagePoint();
					double hemline = sqrt((gpt1.x - gpt2.x)*(gpt1.x - gpt2.x) + (gpt1.y - gpt2.y)*(gpt1.y - gpt2.y));
					double small_quantity = 1e-8;
					double k1 = (gpt1.y - gpt2.y) / (hemline + small_quantity);
					double k2 = (gpt1.x - gpt2.x) / (hemline + small_quantity);

					//for the first endpoint of a line
					residue(c++) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt1.x + k2 * newDpt1.y;

					for(int p=0;p<np;++p)
					{
						imDerp[p] = getForwardDeriv( p , tieFeatureList[i].m_TiePoints[0].getGroundPoint() , pstep_scale);
					}
					//compute influence of tie point on all system elements
					for(int p1=0;p1<np;++p1)
					{        
						//proj residue: J * residue
						projResidue.element(p1) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * residue(c - 1);
						//normal matrix A = transpose(J)*J
						for(int p2=p1;p2<np;++p2)
						{
							A.element(p1,p2) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * (k1 * imDerp[p2].x - k2 * imDerp[p2].y);
						}
					}

					//for the second endpoint of a line

					ossimDpt newDpt2 = forward(tieFeatureList[i].m_TiePoints[1].getGroundPoint());
					residue(c++) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt2.x + k2 * newDpt2.y;
					for(int p=0;p<np;++p)
					{
						imDerp[p] = getForwardDeriv( p , tieFeatureList[i].m_TiePoints[1].getGroundPoint() , pstep_scale);
					}
					//compute influence of tie point on all system elements
					for(int p1=0;p1<np;++p1)
					{        
						//proj residue: J * residue
						projResidue.element(p1) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * residue(c - 1);
						//normal matrix A = transpose(J)*J
						for(int p2=p1;p2<np;++p2)
						{
							A.element(p1,p2) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * (k1 * imDerp[p2].x - k2 * imDerp[p2].y);
						}
					}
					break;
				}
				*/
			
			default:
				{

				}
			}
		}
		//delete []imDerp;
	}
	else
	{
	} //end of if (useImageObs)
}
ossimGpt
ossimSensorModel::getInverseDeriv(int parmIdx, const ossimDpt& ipos, double hdelta)
{   
   double den = 0.5/hdelta;
   ossimGpt res,gd;
   double middle = getAdjustableParameter(parmIdx);
   setAdjustableParameter(parmIdx, middle + hdelta, true);
   //res = inverse_do(ipos,ossimGpt(ipos.lon, ipos.lat, 0.0));
   res = inverse(ipos);
   setAdjustableParameter(parmIdx, middle - hdelta, true);
   gd = inverse(ipos);
   //gd = inverse_do(ipos,ossimGpt(ipos.lon, ipos.lat, 0.0));
   setAdjustableParameter(parmIdx, middle, true);
   res.lon = den*(res.lon - gd.lon) * 100000.0; //TBC : approx meters
   res.lat = den*(res.lat - gd.lat) * 100000.0 * cos(gd.lat / 180.0 * M_PI);
   res.hgt = den*(res.hgt - gd.hgt);

	//double den = 0.5/hdelta;
	//ossimGpt res(0,0),gd;
	//ossimDpt dptres,dptgd;
	//double middle = getAdjustableParameter(parmIdx);
	////set parm to high value
	//setAdjustableParameter(parmIdx, middle + hdelta, true);
	//ossimGpt gpt;
	//res = inverse(ipos);
	////set parm to low value and gte difference
	//setAdjustableParameter(parmIdx, middle - hdelta, true);
	//gd = inverse(ipos);

	////reset parm
	//setAdjustableParameter(parmIdx, middle, true);
	//if(!m_proj) return res;
	//dptres=m_proj->forward(res);
	//dptgd=m_proj->forward(gd);
	//res.lon = den*(dptres.lon - dptgd.lon); //TBC : approx meters
	//res.lat = den*(dptres.lat - dptgd.lat);// * 100000.0 * cos(gd.lat / 180.0 * M_PI);
	//res.hgt = den*(res.hgt - gd.hgt);

   return res;
}

//give forward() partial derivative regarding parameter parmIdx (>=0)
ossimDpt
ossimSensorModel::getForwardDeriv(int parmIdx, const ossimGpt& gpos, double hdelta)
{   
   double den = 0.5/hdelta;
   ossimDpt res;
   double middle = getAdjustableParameter(parmIdx);
   setAdjustableParameter(parmIdx, middle + hdelta, true);
   //res = inverse(gpos);
   res = forward(gpos);
   setAdjustableParameter(parmIdx, middle - hdelta, true);
   //res -= inverse(gpos);
   res -= forward(gpos);
   res = res*den;
   setAdjustableParameter(parmIdx, middle, true);
   return res;
}

NEWMAT::ColumnVector
ossimSensorModel::getResidue(const ossimTieGptSet& tieSet, bool useImageObs/* = true*/)
{
   NEWMAT::ColumnVector residue;
   int dimObs;
   //bool useImageObs = useForward(); //caching
   if (useImageObs)
   {
      dimObs = 2; //image observation
   } else {
      dimObs = 3; //ground observations
   }
   int no = dimObs * tieSet.size(); //number of observations
   residue.ReSize(no);
   const vector<ossimRefPtr<ossimTieGpt> >& theTPV = tieSet.getTiePoints();
   vector<ossimRefPtr<ossimTieGpt> >::const_iterator tit;
   unsigned long c=1;
   if (useImageObs)
   { 
     ossimDpt resIm;
      for (tit = theTPV.begin() ; tit != theTPV.end() ; ++tit)
      {
         resIm = (*tit)->tie - forward(**tit);
         residue(c++) = resIm.x;
         residue(c++) = resIm.y;
      }
   }
   else {
	   if (!m_proj) return residue;
	   ossimGpt gd;
	   ossimDpt tmplast,tmpnew;	// loong
      for (tit = theTPV.begin() ; tit != theTPV.end() ; ++tit)
      {	
		  //loong
         //gd = inverse((*tit)->tie);
		 gd = inverse((*tit)->tie);
		 tmplast=m_proj->forward((*tit)->getGroundPoint());
		 tmpnew=m_proj->forward(gd );
		 residue(c++) =  tmplast.lon-tmpnew.lon;
		 residue(c++) = tmplast.lat-tmpnew.lat;
		 residue(c++) =  (*tit)->hgt - gd.hgt; //TBD : normalize to meters?
		 //
         //residue(c++) = ((*tit)->lon - gd.lon) * 100000.0; //approx meters //TBC TBD
         //residue(c++) = ((*tit)->lat - gd.lat) * 100000.0 * cos(gd.lat / 180.0 * M_PI);
         //residue(c++) = (*tit)->hgt - gd.hgt; //meters
      }
   } //end of if (useImageObs)
   return residue;
}

/*!
 * solves Ax = r , with A symmetric positive definite
 * A can be rank deficient
 * size of A is typically between 10 and 100 rows
 */
NEWMAT::ColumnVector 
ossimSensorModel::solveLeastSquares(NEWMAT::SymmetricMatrix& A,  NEWMAT::ColumnVector& r)const
{
   NEWMAT::ColumnVector x = invert(A)*r;
   return x;
}

/** 
 * stable invert stolen from ossimRpcSolver
 */
NEWMAT::Matrix 
ossimSensorModel::invert(const NEWMAT::Matrix& m)const
{
   ossim_uint32 idx = 0;
   NEWMAT::DiagonalMatrix d;
   NEWMAT::Matrix u;
   NEWMAT::Matrix v;

   // decompose m.t*m which is stored in Temp into the singular values and vectors.
   //
   NEWMAT::SVD(m, d, u, v, true, true);
   
   // invert the diagonal
   // this is just doing the reciprical fo all diagonal components and store back int
   // d.  ths compute d inverse.
   //
   for(idx=0; idx < (ossim_uint32)d.Ncols(); ++idx)
   {
      if(d[idx] > 1e-14) //TBC : use DBL_EPSILON ?
      {
         d[idx] = 1.0/d[idx];
      }
      else
      {
         d[idx] = 0.0;

//DEBUG TBR
cout<<"warning: singular matrix in SVD"<<endl;

      }
   }

   //compute inverse of decomposed m;
   return v*d*u.t();
}

//loong

// 2010.1.14 loong
ossimDpt ossimSensorModel::getCoordinateForwardDeriv(int parmIdx , const ossimGpt& gpos, double hdelta) const
{   
	double den = 0.5 / hdelta;
	ossimDpt res;
	ossimGpt gpt;
	if(0 == parmIdx)
	{//X
		gpt = gpos;
		double middle = gpos.lat;
		//set parm to high value
		gpt.lat = middle + 1.0 * hdelta;
		res = forward(gpt);
		//set parm to low value and gte difference
		gpt.lat = middle - 1.0 * hdelta;
		res -= forward(gpt);
		gpt.lat = middle;
		//get partial derivative
		res = res*den;
	}
	else if(1 == parmIdx)
	{//Y
		gpt = gpos;
		double middle = gpos.lon;
		//set parm to high value
		gpt.lon = middle + 1.0 * hdelta;
		res = forward(gpt);
		//set parm to low value and gte difference
		gpt.lon = middle - 1.0 * hdelta;
		res -= forward(gpt);
		gpt.lon = middle;
		//get partial derivative
		res = res*den;
	}
	else if(2 == parmIdx)
	{//Z
		gpt = gpos;
		double middle = gpos.hgt;
		//set parm to high value
		gpt.hgt = middle + 1.0 * hdelta;
		res = forward(gpt);
		//set parm to low value and gte difference
		gpt.hgt = middle - 1.0 * hdelta;
		res -= forward(gpt);
		gpt.hgt = middle;
		//get partial derivative
		res = res*den;
	}
	return res;
}

int ossimSensorModel::getNumofObservations(const vector< ossimTieFeature >& tieFeatureList, bool useImageObs)
{
	int no = 0;
	for(unsigned int i = 0 ;i < tieFeatureList.size();i++)
	{
		switch(tieFeatureList[i].getTieFeatureType())
		{
		case  ossimTieFeature::ossimTiePointPoint:
			{
				if (useImageObs)
				{
					no += 2; //image observation
				} else {
					no += 3; //ground observations
				}
				break;
			}
		case ossimTieFeature::ossimTieLineLine:
			{
				no += 2;
				break;
			}
		case ossimTieFeature::ossimTieAreaArea:
			{
				no += 2;
				break;
			}
		case  ossimTieFeature::ossimTieLinePoint:
			{
				break;
			}
		case ossimTieFeature::ossimTieAreaLine:
			{
				break;
			}
		case ossimTieFeature::ossimTiePointArea:
			{
				break;
			}
		case  ossimTieFeature::ossimTieAreaPoint:
			{
				break;
			}
		case ossimTieFeature::ossimTiePointLine:
			{
				break;
			}
		case ossimTieFeature::ossimTieLineArea:
			{
				break;
			}
		default:
			{
				break;
			}
		}
	}
	return no;
}

NEWMAT::ColumnVector ossimSensorModel::getResidue(const vector< ossimTieFeature >& tieFeatureList, bool useImageObs/* = true  2010.1.18 loong*/)
{

	//init
	NEWMAT::ColumnVector residue;
	int dimObs;
	ossimDpt tmplast,tmpnew;
	//bool useImageObs = useForward(); //caching
	if (useImageObs)
	{
		dimObs = 2; //image observation
	} else {
		dimObs = 3; //ground observations
	}
	int nobs = tieFeatureList.size();
	//int no = dimObs * nobs; //number of observations
	int no = getNumofObservations(tieFeatureList, useImageObs);
	residue.ReSize(no);
	unsigned long c=1;
	if(!m_proj) return residue;
	int i;
	if (useImageObs)
	{ 
		// loop on tie points
		for (i = 0 ; i < nobs ; ++i)
		{
			switch(tieFeatureList[i].getTieFeatureType())
			{
			case  ossimTieFeature::ossimTiePointPoint:
				{
					//image observations 
					ossimDpt resIm;
					resIm = tieFeatureList[i].getImageFeature().m_Points[0] - forward(tieFeatureList[i].getGroundFeature().m_Points[0]);
					residue(c++) = resIm.x;
					residue(c++) = resIm.y;
					break;
				}
			case ossimTieFeature::ossimTieLineLine:
				{
					ossimDpt newDpt1 = forward(tieFeatureList[i].getGroundFeature().m_Points[0]);
					ossimDpt newDpt2 = forward(tieFeatureList[i].getGroundFeature().m_Points[1]);
					ossimDLine newDLine(newDpt1, newDpt2);
					ossimDLine oldDLine(tieFeatureList[i].getImageFeature().m_Points[0], tieFeatureList[i].getImageFeature().m_Points[1]);
					ossimDpt res = oldDLine.DistanceAndSegment(newDLine);

					residue(c++) = res.x;
					residue(c++) = res.y;

					//ossimDpt newDpt1 = forward(tieFeatureList[i].getGroundFeature().m_Points[0]);
					//ossimDpt newDpt2 = forward(tieFeatureList[i].getGroundFeature().m_Points[1]);
					//ossimDpt gpt1 = tieFeatureList[i].getImageFeature().m_Points[0];
					//ossimDpt gpt2 = tieFeatureList[i].getImageFeature().m_Points[1];
					//double hemline = sqrt((gpt1.x - gpt2.x)*(gpt1.x - gpt2.x) + (gpt1.y - gpt2.y)*(gpt1.y - gpt2.y));
					//double k1 = (gpt1.y - gpt2.y) / (hemline + small_quantity);
					//double k2 = (gpt1.x - gpt2.x) / (hemline + small_quantity);

					//residue(c++) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt1.x + k2 * newDpt1.y;
					//residue(c++) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt2.x + k2 * newDpt2.y;
					break;
				}
			case ossimTieFeature::ossimTieAreaArea:
				{
					vector<ossimDpt> newPoints;
					for(unsigned int iVertex = 0;iVertex < tieFeatureList[i].getGroundFeature().m_Points.size();iVertex++)
					{
						newPoints.push_back(forward(tieFeatureList[i].getGroundFeature().m_Points[iVertex]));
					}
					ossimDArea oldArea(tieFeatureList[i].getImageFeature().m_Points);
					ossimDArea newArea(newPoints);	
					ossimDpt tmpDisVector;
					oldArea.DistanceFromArea(newArea, &tmpDisVector);
					//ossimDpt tmpDisVector = oldArea.DistanceFromAreaMid(newArea);
					residue(c++) = -tmpDisVector.x;
					residue(c++) = -tmpDisVector.y;
					break;
				}
				/*
			case ossimTieFeature::ossimFeatureType::ossimGptFeature:
				{
					//image observations 
					ossimDpt resIm;
					resIm = tieFeatureList[i].m_TiePoints[0].getImagePoint() - forward(tieFeatureList[i].m_TiePoints[0].getGroundPoint());
					residue(c++) = resIm.x;
					residue(c++) = resIm.y;
					break;
				}
			case ossimTieFeature::ossimFeatureType::ossimLineFeature:
				{
					//compute residue
					ossimDpt newDpt1 = forward(tieFeatureList[i].m_TiePoints[0].getGroundPoint());
					ossimDpt newDpt2 = forward(tieFeatureList[i].m_TiePoints[1].getGroundPoint());
					ossimDpt gpt1 = tieFeatureList[i].m_TiePoints[0].getImagePoint();
					ossimDpt gpt2 = tieFeatureList[i].m_TiePoints[1].getImagePoint();
					double hemline = sqrt((gpt1.x - gpt2.x)*(gpt1.x - gpt2.x) + (gpt1.y - gpt2.y)*(gpt1.y - gpt2.y));
					double small_quantity = 1e-8;
					double k1 = (gpt1.y - gpt2.y) / (hemline + small_quantity);
					double k2 = (gpt1.x - gpt2.x) / (hemline + small_quantity);

					residue(c++) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt1.x + k2 * newDpt1.y;
					residue(c++) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt2.x + k2 * newDpt2.y;
					break;
				}
				*/
			default:
				{

				}
			}
		}
	}
	else {
		// ground observations
	} //end of if (useImageObs)
	return residue;
}

double ossimSensorModel::calcHeight(const ossimTieFeature& tieFeature) const
{
	//setup initail values
	ossimTieFeature local_tieFeature(tieFeature);
	int iter=0;
	int iter_max = 50;
	double minResidue = 1e-10; //TBC
	double minDelta = 1e-10; //TBC
	NEWMAT::ColumnVector residue;
	double a;
	double b;
	double deltap_scale = 1e-4;
	calcHeight_buildNormalEquation(local_tieFeature, a, residue, b, deltap_scale);

	double ki2=residue.SumSquare();
	double damping_speed = 2.0;
	//find max diag element for A
	double maxdiag = a;
	double damping = 1e-3 * maxdiag;
	double olddamping = 0.0;
	bool found = false;

	while ( (!found) && (iter < iter_max) ) //non linear optimization loop
	{
		bool decrease = false;

		do
		{
			//add damping update to normal matrix
			a += damping - olddamping;
			olddamping = damping;
			double deltaZ = b / (a + DBL_EPSILON);

			if (deltaZ <= minDelta) 
			{
				found = true;
			} else {
				//update adjustment
				for(size_t ii = 0;ii < local_tieFeature.getGroundFeature().m_Points.size();ii++)
				{
					local_tieFeature.refGroundFeature().m_Points[ii].hgt += deltaZ;
				}

				//check residue is reduced
				NEWMAT::ColumnVector newresidue = calcHeight_getResidue(local_tieFeature);
				double newki2=newresidue.SumSquare();
				double res_reduction = (ki2 - newki2) / (deltaZ*(deltaZ*damping + b));
				////DEBUG TBR
				//cout<<sqrt(newki2/2)<<" ";
				//cout.flush();

				if (res_reduction > 0)
				{
					//accept new parms
					ki2=newki2;
					calcHeight_buildNormalEquation(local_tieFeature, a, residue, b, deltap_scale);
					olddamping = 0.0;

					found = ( b <= minResidue );
					//update damping factor
					damping *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
					damping_speed = 2.0;
					decrease = true;
				} else {
					//cancel parameter update
					for(size_t ii = 0;ii < local_tieFeature.getGroundFeature().m_Points.size();ii++)
					{
						local_tieFeature.refGroundFeature().m_Points[ii].hgt -= deltaZ;
					}

					damping *= damping_speed;
					damping_speed *= 2.0;
				}
			}
		} while (!decrease && !found);
		++iter;
	}

	double sumH = 0.0;
	double sumH0 = 0.0;
	for(size_t ii = 0;ii < local_tieFeature.getGroundFeature().m_Points.size();ii++)
	{
		sumH += local_tieFeature.getGroundFeature().m_Points[ii].hgt;
		sumH0 += ossimElevManager::instance()->getHeightAboveEllipsoid(local_tieFeature.getGroundFeature().m_Points[ii]);
	}
	sumH /= local_tieFeature.getGroundFeature().m_Points.size();
	sumH0 /= local_tieFeature.getGroundFeature().m_Points.size();
	return sumH - sumH0;
}

void ossimSensorModel::calcHeight_buildNormalEquation(const ossimTieFeature& tieFeature,
													 double& a,
													 NEWMAT::ColumnVector& residue,
													 double& b,
													 double pstep_scale)const
{
	int index=0;
	switch(tieFeature.getTieFeatureType())
	{
	case ossimTieFeature::ossimTieFeatureType::ossimTiePointPoint:
		{
			// 点-点 列两个方程
			residue.ReSize(2);
			ossimDpt resIm = tieFeature.getImageFeature().m_Points[0] - forward(tieFeature.getGroundFeature().m_Points[0]);
			residue(0) = resIm.x;
			residue(1) = resIm.y;

			// 对高程求导
			ossimDpt imDerp = getCoordinateForwardDeriv(2 , tieFeature.getGroundFeature().m_Points[0], pstep_scale);

			a = imDerp.x * imDerp.x + imDerp.y * imDerp.y;
			b = imDerp.x * resIm.x + imDerp.y * resIm.y;
			break;
		}
	case ossimTieFeature::ossimTieFeatureType::ossimTieLineLine:
		{
			// 线-线 列两个方程
			residue.ReSize(2);
			ossimDpt newPoint1 = forward(tieFeature.getGroundFeature().m_Points[0]);
			ossimDpt newPoint2 = forward(tieFeature.getGroundFeature().m_Points[1]);

			ossimDLine oldLine(tieFeature.getImageFeature().m_Points[0],
				tieFeature.getImageFeature().m_Points[1]);

			// 对第一个端点列方程
			double d1 = oldLine.DistanceFromPoint(newPoint1);
			residue(0) = d1;
			// 距离X函数对像素坐标微分
			double patialDistance_X;
			// 距离Y函数对像素坐标微分
			double patialDistance_Y;
			oldLine.getPointDistanceDeriv(newPoint1, &patialDistance_X, &patialDistance_Y, pstep_scale);

			// 对高程求导
			ossimDpt imDerp = getCoordinateForwardDeriv(2 , tieFeature.getGroundFeature().m_Points[0], pstep_scale);

			// 对第一个端点的导数
			double derp1 = patialDistance_X * imDerp.x + patialDistance_Y * imDerp.y;

			// 对第二个端点列方程
			double d2 = oldLine.DistanceFromPoint(newPoint2);
			residue(1) = d2;
			oldLine.getPointDistanceDeriv(newPoint2, &patialDistance_X, &patialDistance_Y, pstep_scale);

			// 对高程求导
			imDerp = getCoordinateForwardDeriv(2 , tieFeature.getGroundFeature().m_Points[1], pstep_scale);

			// 对第二个端点的导数
			double derp2 = patialDistance_X * imDerp.x + patialDistance_Y * imDerp.y;

			a = derp1 * derp1 + derp2 * derp2;
			b = derp1 * d1 + derp2 * d2;
			break;
		}
	case ossimTieFeature::ossimTieFeatureType::ossimTieAreaArea:
		{

			break;
		}
	default:
		{

		}
	}
}

NEWMAT::ColumnVector ossimSensorModel::calcHeight_getResidue(const ossimTieFeature& tieFeature)const
{
	int index=0;
	NEWMAT::ColumnVector residue;
	switch(tieFeature.getTieFeatureType())
	{
	case ossimTieFeature::ossimTieFeatureType::ossimTiePointPoint:
		{
			// 点-点 列两个方程
			residue.ReSize(2);
			ossimDpt resIm = tieFeature.getImageFeature().m_Points[0] - forward(tieFeature.getGroundFeature().m_Points[0]);
			residue(0) = resIm.x;
			residue(1) = resIm.y;
			break;
		}
	case ossimTieFeature::ossimTieFeatureType::ossimTieLineLine:
		{
			// 线-线 列两个方程
			residue.ReSize(2);
			ossimDpt newPoint1 = forward(tieFeature.getGroundFeature().m_Points[0]);
			ossimDpt newPoint2 = forward(tieFeature.getGroundFeature().m_Points[1]);

			ossimDLine oldLine(tieFeature.getImageFeature().m_Points[0],
				tieFeature.getImageFeature().m_Points[1]);

			// 对第一个端点
			double d1 = oldLine.DistanceFromPoint(newPoint1);
			residue(0) = d1;

			// 对第二个端点
			double d2 = oldLine.DistanceFromPoint(newPoint2);
			residue(1) = d2;
			break;
		}
	case ossimTieFeature::ossimTieFeatureType::ossimTieAreaArea:
		{

			break;
		}
	default:
		{

		}
	}
	return residue;
}

int lmder_functor::operator()(const VectorXd &x, VectorXd &fvec) const
{
	optimizeStruct *pOptimizeStruct = (optimizeStruct*)pData;

	int np = pOptimizeStruct->pThis->getNumberOfAdjustableParameters();
	int nobs = pOptimizeStruct->tieFeatureList.size();

	for(int n=0;n<np;++n)
	{
		pOptimizeStruct->pThis->setAdjustableParameter(n, x[n], false);
	}
	pOptimizeStruct->pThis->updateModel();

	double pstep_scale = 1e-4;
	static double den = 0.5 / pstep_scale;

	int c = 0;
	for (int i = 0;i < nobs;++i)
	{
		std::vector<ossimDpt> imDerp(np);
		ossimDpt resIm;
		for (i = 0 ; i < nobs ; ++i)
		{
			switch(pOptimizeStruct->tieFeatureList[i].getTieFeatureType())
			{
			case ossimTieFeature::ossimTieFeatureType::ossimTiePointPoint:
				{
					resIm = pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[0] - pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);
					fvec[c++] = -resIm.x;
					fvec[c++] = -resIm.y;
					break;
				}
			case ossimTieFeature::ossimTieFeatureType::ossimTieLineLine:
				{
					//ossimDpt newDpt1 = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);
					//ossimDpt newDpt2 = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1]);
					//ossimDLine newDLine(newDpt1, newDpt2);
					//ossimDLine oldDLine(pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[0], pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[1]);
					////ossimDpt newLinePoint, oldLinePoint;
					////newDLine.toPoint(&newLinePoint);
					////oldDLine.toPoint(&oldLinePoint);
					////ossimDpt res = newLinePoint - oldLinePoint;
					//ossimDpt res = oldDLine.DistanceFromSegment(newDLine);
					//fvec[c++] = res.x;
					//fvec[c++] = res.y;

					ossimDpt newDpt1 = pOptimizeStruct->pThis->forward( pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);
					ossimDpt newDpt2 = pOptimizeStruct->pThis->forward( pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1]);
					ossimDLine newDLine(newDpt1, newDpt2);
					ossimDLine oldDLine( pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[0],  pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[1]);
					ossimDpt res = oldDLine.DistanceAndSegment(newDLine);

					fvec[c++] = res.x;
					fvec[c++] = res.y;
					break;
				}
			case ossimTieFeature::ossimTieFeatureType::ossimTieAreaArea:
				{
					//
					vector<ossimDpt> newPoints;
					for(unsigned int iVertex = 0;iVertex < pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points.size();iVertex++)
					{
						newPoints.push_back(pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[iVertex]));
					}
					ossimDArea oldArea(pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points);
					ossimDArea newArea(newPoints);	
					int nIndex;
					ossimDpt tmpDisVector;
					oldArea.DistanceFromArea(newArea, &tmpDisVector);
					//ossimDpt tmpDisVector = oldArea.DistanceFromAreaMid(newArea, &nIndex);
					fvec[c++] = tmpDisVector.x;
					fvec[c++] = tmpDisVector.y;
					break;
				}
			default:
				{

				}
			}
		}
	}
	return 0;
}

int lmder_functor::df(const VectorXd &x, MatrixXd &fjac) const
{
	optimizeStruct *pOptimizeStruct = (optimizeStruct*)pData;

	int np = pOptimizeStruct->pThis->getNumberOfAdjustableParameters();
	int nobs = pOptimizeStruct->tieFeatureList.size();

	for(int n=0;n<np;++n)
	{
		pOptimizeStruct->pThis->setAdjustableParameter(n, x[n], false);
	}
	pOptimizeStruct->pThis->updateModel();

	double pstep_scale = 1e-4;
	static double den = 0.5 / pstep_scale;

	std::vector<ossimDpt> imDerp(np);
	ossimDpt resIm;
	int c = 0;
	for (int i = 0;i < nobs;++i,c+=2)
	{
		switch(pOptimizeStruct->tieFeatureList[i].getTieFeatureType())
		{
		case ossimTieFeature::ossimTieFeatureType::ossimTiePointPoint:
			{
				resIm = pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[0] - pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);

				//compute all image derivatives regarding parametres for the tie point position
				for(int p=0;p<np;++p)
				{
					imDerp[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0], pstep_scale);

					fjac(c,p) = imDerp[p].x;
					fjac(c+1,p) = imDerp[p].y;
				}
				break;
			}
		case ossimTieFeature::ossimTieFeatureType::ossimTieLineLine:
			{
				//ossimDpt newDpt1 = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);
				//ossimDpt newDpt2 = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1]);
				//ossimDLine newDLine(newDpt1, newDpt2);
				//ossimDLine oldDLine(pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[0], pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[1]);
				////ossimDpt newLinePoint, oldLinePoint;
				////newDLine.toPoint(&newLinePoint);
				////oldDLine.toPoint(&oldLinePoint);
				////ossimDpt res = newLinePoint - oldLinePoint;

				//for(int p=0;p<np;++p)
				//{
				//	double middle = pOptimizeStruct->pThis->getAdjustableParameter(p);

				//	pOptimizeStruct->pThis->setAdjustableParameter(p, middle + pstep_scale, true);

				//	ossimDpt newDpt1_ = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);
				//	ossimDpt newDpt2_ = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1]);
				//	ossimDLine newDLine_(newDpt1_, newDpt2_);
				//	ossimDpt dist1;
				//	newDLine_.toPoint(&dist1);

				//	pOptimizeStruct->pThis->setAdjustableParameter(p, middle - pstep_scale, true);
				//	newDpt1_ = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);
				//	newDpt2_ = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1]);
				//	newDLine_.setPoints(newDpt1_, newDpt2_);
				//	ossimDpt dist2;
				//	newDLine_.toPoint(&dist2);

				//	pOptimizeStruct->pThis->setAdjustableParameter(p, middle, true);
				//	double derivative_x = (dist1.x - dist2.x) * den;
				//	double derivative_y = (dist1.y - dist2.y) * den;

				//	fjac(c,p) = derivative_x;
				//	fjac(c+1,p) = derivative_y;
				//}

				//vector<ossimDpt> imDerp1(np);
				//vector<ossimDpt> imDerp2(np);
				//for(int p=0;p<np;++p)
				//{
				//	imDerp1[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0], pstep_scale);
				//	imDerp2[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1], pstep_scale);
				//}

				//// 直线X函数对像素坐标微分
				//ossimDpt patialDistance_1;
				//// 直线Y函数对像素坐标微分
				//ossimDpt patialDistance_2;
				//oldDLine.getPointDistanceDeriv(newDLine.getFirstPoint(), &patialDistance_1.x, &patialDistance_1.y);
				//oldDLine.getPointDistanceDeriv(newDLine.getSecondPoint(), &patialDistance_2.x, &patialDistance_2.y);

				//for(int p=0;p<np;++p)
				//{ 
				//	double derivative_1 = patialDistance_1.x * imDerp1[p].x + patialDistance_1.y * imDerp1[p].y;
				//	double derivative_2 = patialDistance_1.x * imDerp[p].x + patialDistance_1.y * imDerp1[p].y;

				//	fjac(c,p) = derivative_1;
				//	fjac(c+1,p) = derivative_2;
				//}


				ossimDpt newDpt1 = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);
				ossimDpt newDpt2 = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1]);
				ossimDLine newDLine(newDpt1, newDpt2);
				ossimDLine oldDLine(pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[0], pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points[1]);
				ossimDpt res = oldDLine.DistanceAndSegment(newDLine);

				//vector<ossimDpt> imDerp1(np);
				//vector<ossimDpt> imDerp2(np);
				//for (int p = 0; p < np; p++)
				//{
				//	imDerp1[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0], pstep_scale);
				//	imDerp2[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1], pstep_scale);
				//	double derv_x1, derv_y1, derv_x2, derv_y2;
				//	oldDLine.getPointDistanceDeriv(newDpt1, &derv_x1, &derv_y1);
				//	oldDLine.getPointDistanceDeriv(newDpt2, &derv_x2, &derv_y2);
				//	double derivative_1 = imDerp1[p].x * derv_x1 + imDerp1[p].y * derv_y1;
				//	double derivative_2 = imDerp2[p].x * derv_x2 + imDerp2[p].y * derv_y2;
				//	fjac(c,p) = derivative_1;
				//	fjac(c+1,p) = derivative_2;
				//}
				for(int p=0;p<np;++p)
				{
					double middle = pOptimizeStruct->pThis->getAdjustableParameter(p);

					pOptimizeStruct->pThis->setAdjustableParameter(p, middle + pstep_scale, true);

					ossimDpt newDpt1_ = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);
					ossimDpt newDpt2_ = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1]);
					ossimDLine newDLine_(newDpt1_, newDpt2_);
					ossimDpt dist1 = oldDLine.DistanceAndSegment(newDLine_);

					pOptimizeStruct->pThis->setAdjustableParameter(p, middle - pstep_scale, true);
					newDpt1_ = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[0]);
					newDpt2_ = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[1]);
					newDLine_.setPoints(newDpt1_, newDpt2_);
					ossimDpt dist2 = oldDLine.DistanceAndSegment(newDLine_);

					pOptimizeStruct->pThis->setAdjustableParameter(p, middle, true);
					double derivative_x = (dist1.x - dist2.x) * den;
					double derivative_y = (dist1.y - dist2.y) * den;

					fjac(c,p) = derivative_x;
					fjac(c+1,p) = derivative_y;
				}
				break;
			}
		case ossimTieFeature::ossimTieFeatureType::ossimTieAreaArea:
			{
				//
				vector<ossimDpt> newPoints;
				for(unsigned int iVertex = 0;iVertex < pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points.size();iVertex++)
				{
					newPoints.push_back(pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[iVertex]));
				}
				ossimDArea oldArea(pOptimizeStruct->tieFeatureList[i].getImageFeature().m_Points);
				ossimDArea newArea(newPoints);
				ossimDpt tmpDisVector;
				oldArea.DistanceFromArea(newArea, &tmpDisVector);
				//ossimDpt tmpDisVector = oldArea.DistanceFromAreaMid(newArea, &nIndex);

				double hdelta = 1.0e-5;
				for(int p=0;p<np;++p)
				{
					static double den = 0.5/hdelta;
					ossimDpt dist1, dist2;
					double middle = pOptimizeStruct->pThis->getAdjustableParameter(p);
					pOptimizeStruct->pThis->setAdjustableParameter(p, middle + hdelta, true);
					for(unsigned int iVertex = 0;iVertex < pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points.size();iVertex++)
					{
						newArea.m_Points[iVertex] = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[iVertex]);
					}
					oldArea.DistanceFromArea(newArea, &dist1);
					pOptimizeStruct->pThis->setAdjustableParameter(p, middle - hdelta, true);
					for(unsigned int iVertex = 0;iVertex < pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points.size();iVertex++)
					{
						newArea.m_Points[iVertex] = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getGroundFeature().m_Points[iVertex]);
					}
					oldArea.DistanceFromArea(newArea, &dist2);
					ossimDpt res = dist1 - dist2;
					res = res*den;
					fjac(c,p) = res.x;
					fjac(c+1,p) = res.y;
				}
				break;
			}			
		default:
			{

			}
		}
	}
	return 0;
}