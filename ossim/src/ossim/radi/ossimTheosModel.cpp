//*******************************************************************
//
// License:  See top level LICENSE.txt file.
//
// Author:  Long Tengfei
//
// Description:
//
// Contains definition of class ossimTheosModel.
//
//*****************************************************************************
// $Id: ossimTheosModel.cpp 19658 2014-03-01  $

#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

#include <ossim/radi/ossimTheosModel.h>
#include <ossim/radi/ossimTheosDimapSupportData.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/projection/ossimMapProjection.h>
#include <ossim/base/ossimLsrPoint.h>
#include <ossim/base/ossimEcefRay.h>
#include <ossim/base/ossimLsrRay.h>
#include <ossim/base/ossimLsrSpace.h>
#include <ossim/base/ossimDpt3d.h>
#include <ossim/base/ossimColumnVector3d.h>
#include <ossim/base/ossimNotifyContext.h>

RTTI_DEF1(ossimTheosModel, "ossimTheosModel", ossimSensorModel);


//---
// Define Trace flags for use within this file:
//---
#include <ossim/base/ossimTrace.h>
static ossimTrace traceExec  ("ossimTheosModel:exec");
static ossimTrace traceDebug ("ossimTheosModel:debug");

static const ossim_int32 MODEL_VERSION_NUMBER = 1;

static const char* PARAM_NAMES[] = { "roll_offset",
                                     "pitch_offset",
                                     "yaw_offset",
                                     "roll_rate",
                                     "pitch_rate",
                                     "yaw_rate",
                                     "focal_length_offset" };

static const char* PARAM_UNITS[] = { "degrees",   // degrees
                                     "degrees",   // degrees
                                     "degrees",   // degrees
                                     "degrees",   // degrees/sec
                                     "degrees",   // degrees/sec
                                     "degrees",   // degrees/sec
                                     "unknown" }; // percent deviation from nominal

static const ossim_float64 SIGMA[] = { 0.0001,   // degrees
                                       0.0001,   // degrees
                                       0.0003,   // degrees
                                       0.00002,  // delta degrees
                                       0.00002,  // delta degrees
                                       0.00005,  // delta degrees
                                       0.0001 }; // percent

ossimTheosModel::ossimTheosModel()
   :
   ossimSensorModel      (),
   theSupportData        (NULL),
   theMetaDataFile       ("NOT ASSIGNED"),
   theIllumAzimuth       (0.0),
   theIllumElevation     (0.0),
   thePositionError      (0.0),
   theRefImagingTime     (0.0),
   theRefImagingTimeLine (0.0),
   theLineSamplingPeriod (0.0),
//   theSatToOrbRotation   (3, 3),
//   theOrbToEcfRotation   (3, 3),
   theRollOffset         (0.0),
   thePitchOffset        (0.0),
   theYawOffset          (0.0),
   theRollRate           (0.0),
   thePitchRate          (0.0),
   theYawRate            (0.0),
   theFocalLenOffset     (0.0)
{
   initAdjustableParameters();
}

ossimTheosModel::ossimTheosModel(ossimTheosDimapSupportData* sd)
   :
   ossimSensorModel      (),
   theSupportData        (sd),
   theMetaDataFile       ("NOT ASSIGNED"),
   theIllumAzimuth       (0.0),
   theIllumElevation     (0.0),
   thePositionError      (0.0),
   theRefImagingTime     (0.0),
   theRefImagingTimeLine (0.0),
   theLineSamplingPeriod (0.0),
//   theSatToOrbRotation   (3, 3),
//   theOrbToEcfRotation   (3, 3),
   theRollOffset         (0.0),
   thePitchOffset        (0.0),
   theYawOffset          (0.0),
   theRollRate           (0.0),
   thePitchRate          (0.0),
   theYawRate            (0.0),
   theFocalLenOffset     (0.0)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel(dimap_file) Constructor: entering..." << std::endl;

   //---
   // Instantiate the support data classes after establishing the filenames:
   //---
   loadSupportData();
   if (getErrorStatus() != ossimErrorCodes::OSSIM_OK)
   {
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel(dimap_file) Constructor: returning with error..." << std::endl;
      return;
   }

   //---
   // initialize remaining data members:
   //---
   initAdjustableParameters();
   updateModel();

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel(dimap_file) Constructor: returning..." << std::endl;
}

//*****************************************************************************
//  DESTRUCTOR: ~ossimTheosModel()
//
//*****************************************************************************
ossimTheosModel::~ossimTheosModel()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG DESTRUCTOR: ~ossimTheosModel(): entering..." << std::endl;

   theSupportData = 0;
   
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG DESTRUCTOR: ~ossimTheosModel(): returning..." << std::endl;
}

ossimTheosModel::ossimTheosModel(const ossimTheosModel& rhs)
   :ossimSensorModel(rhs)
{
   if(rhs.theSupportData.valid())
   {
      theSupportData = (ossimTheosDimapSupportData*)rhs.theSupportData->dup();
   }
   loadSupportData();
   updateModel();
}

#if 1
void ossimTheosModel::computeSatToOrbRotation(NEWMAT::Matrix& result, ossim_float64 t)const
{
   if (traceExec())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
      << "DEBUG ossimTheosModel::computeSatToOrbRotation(): entering..."
      << std::endl;
   }   
   
   // bias
   //---
   // Linearly interpolate attitudes angles:
   //---
   ossimDpt3d attitude_bias = theSupportData->getAttitudeBias();

   //---
   // Apply the attitude adjustable parameters:
   //---
   double dt = theRefImagingTime - t;
   attitude_bias.x     += thePitchOffset + dt*thePitchRate;
   attitude_bias.y     += theRollOffset  + dt*theRollRate;
   attitude_bias.z     += theYawOffset   + dt*theYawRate;

   //---
   // Compute trig functions to populate rotation matrices: ANGLES IN RADIANS
   //---
   double cp = cos(attitude_bias.x);
   double sp = sin(attitude_bias.x);
   double cr = cos(attitude_bias.y);
   double sr = sin(attitude_bias.y);
   double cy = cos(attitude_bias.z);
   double sy = sin(attitude_bias.z);

   //---
   // Populate rotation matrix:
   //---
   NEWMAT::Matrix matBias;
   matBias = NEWMAT::Matrix(3,3);
   matBias << (cr*cy) << (-cr*sy) << (-sr)
	   << (cp*sy+sp*sr*cy) << (cp*cy-sp*sr*sy) << (sp*cr)
	   << (-sp*sy+cp*sr*cy) << (-sp*cy-cp*sr*sy) <<  cp*cr;


   //---
   // Linearly interpolate attitudes angles:
   //---
   std::vector<ossim_float64> q;
   theSupportData->getAttitude(t, q);


   //---
   // Apply the attitude adjustable parameters:
   //---
   q[1] += thePitchOffset;
   q[2] += theRollOffset;
   q[3] += theYawOffset;
   ossim_float64 tmp = sqrt(1.0 - q[1]*q[1]
   -q[2]*q[2] - q[3]*q[3]);
   q[0] = q[0] < 0 ? -tmp : tmp;

   //---
   // Compute trig functions to populate rotation matrices: ANGLES IN RADIANS
   //---
   //double q1 = q[0];
   //double q2 = q[1];
   //double q3 = q[2];
   //double q4 = q[3];
   //double R11 = q1*q1 + q2*q2 - q3*q3 - q4*q4;
   //double R12 = 2.0*(q2*q3 - q1*q4);
   //double R13 = 2.0*(q2*q4 + q1*q3);
   //double R21 = 2.0*(q2*q3 + q1*q4);
   //double R22 = q1*q1 - q2*q2 + q3*q3 - q4*q4;
   //double R23 = 2.0*(q3*q4 - q1*q2);
   //double R31 = 2.0*(q2*q4 - q1*q3);
   //double R32 = 2.0*(q3*q4 + q1*q2);
   //double R33 = q1*q1 - q2*q2 - q3*q3 + q4*q4;

   //double R11 = q1*q1 - q2*q2 - q3*q3 + q4*q4;
   //double R12 = 2.0*(q1*q2 + q3*q4);
   //double R13 = 2.0*(q1*q3 - q2*q4);
   //double R21 = 2.0*(q1*q2 - q3*q4);
   //double R22 = -q1*q1 + q2*q2 - q3*q3 + q4*q4;
   //double R23 = 2.0*(q2*q3 + q1*q4);
   //double R31 = 2.0*(q1*q3 + q2*q4);
   //double R32 = 2.0*(q2*q3 - q1*q4);
   //double R33 = -q1*q1 - q2*q2 + q3*q3 + q4*q4;

   double q0 = q[0];
   double q1 = q[1];
   double q2 = q[2];
   double q3 = q[3];
   //double R11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
   //double R12 = 2.0*(q1*q2 - q0*q3);
   //double R13 = 2.0*(q0*q2 + q1*q3);
   //double R21 = 2.0*(q1*q2 + q0*q3);
   //double R22 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
   //double R23 = 2.0*(q2*q3 - q0*q1);
   //double R31 = 2.0*(q1*q3 - q0*q2);
   //double R32 = 2.0*(q0*q1 + q2*q3);
   //double R33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

   double R11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
   double R12 = 2.0*(q1*q2 - q0*q3);
   double R13 = 2.0*(q0*q2 + q1*q3);
   double R21 = 2.0*(q1*q2 + q0*q3);
   double R22 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
   double R23 = 2.0*(q2*q3 - q0*q1);
   double R31 = 2.0*(q1*q3 - q0*q2);
   double R32 = 2.0*(q0*q1 + q2*q3);
   double R33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

   //R11 = 1.0;
   //R12 = 0.0;
   //R13 = 0.0;
   //R21 = 0.0;
   //R22 = 1.0;
   //R23 = 0.0;
   //R31 = 0.0;
   //R32 = 0.0;
   //R33 = 1.0;

   //---
   // Populate rotation matrix:
   //---
   result = NEWMAT::Matrix(3,3);
   result	<< (R11) << (R12) << (R13)
	   << (R21) << (R22) << (R23)
	   << (R31) << (R32) <<  R33;
   
   result = matBias * result;
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::computeSatToOrbRotation(): returning..." << std::endl;
}

#else
//*****************************************************************************
//  METHOD
//*****************************************************************************
void ossimTheosModel::computeSatToOrbRotation(ossim_float64 t)const
{
   if (traceExec())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "DEBUG ossimTheosModel::computeSatToOrbRotation(): entering..."
         << std::endl;
   }

   //---
   // Linearly interpolate attitudes angles:
   //---
   ossimDpt3d att;
   theSupportData->getAttitude(t, att);

   //---
   // Apply the attitude adjustable parameters:
   //---
   double dt = theRefImagingTime - t;
   att.x     += thePitchOffset + dt*thePitchRate;
   att.y     += theRollOffset  + dt*theRollRate;
   att.z     += theYawOffset   + dt*theYawRate;

   //---
   // Compute trig functions to populate rotation matrices: ANGLES IN RADIANS
   //---
    double cp = cos(att.x);
    double sp = sin(att.x);
    double cr = cos(att.y);
    double sr = sin(att.y);
    double cy = cos(att.z);
    double sy = sin(att.z);

   //---
   // Populate rotation matrix:
   //---
    theSatToOrbRotation = NEWMAT::Matrix(3,3);
    theSatToOrbRotation << (cr*cy) << (-cr*sy) << (-sr)
                        << (cp*sy+sp*sr*cy) << (cp*cy-sp*sr*sy) << (sp*cr)
                        << (-sp*sy+cp*sr*cy) << (-sp*cy-cp*sr*sy) <<  cp*cr;


    if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::computeSatToOrbRotation(): returning..." << std::endl;
}
#endif
//*****************************************************************************
// PUBLIC METHOD: ossimTheosModel::updateModel()
//
//  Updates the model parameters given the normalized adjustable parameter
//  array.
//
//*****************************************************************************
void ossimTheosModel::updateModel()
{
   clearErrorStatus();

   try
   {
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::updateModel(): entering..." << std::endl;

      if(getNumberOfAdjustableParameters() < 1)
      {
         theRollOffset     = 0;
         thePitchOffset    = 0;
         theYawOffset      = 0;
         theRollRate       = 0;
         thePitchRate      = 0;
         theYawRate        = 0;
         theFocalLenOffset = 0;
      }
      else
      {
         theRollOffset     = computeParameterOffset(0);
         thePitchOffset    = computeParameterOffset(1);
         theYawOffset      = computeParameterOffset(2);
         theRollRate       = computeParameterOffset(3);
         thePitchRate      = computeParameterOffset(4);
         theYawRate        = computeParameterOffset(5);
         theFocalLenOffset = computeParameterOffset(6);
      }
      theSeedFunction = 0;
	  ossimGpt ulg, urg, lrg, llg;

      lineSampleToWorld(theImageClipRect.ul(), ulg);
      lineSampleToWorld(theImageClipRect.ur(), urg);
      lineSampleToWorld(theImageClipRect.lr(), lrg);
      lineSampleToWorld(theImageClipRect.ll(), llg);
      theSeedFunction = new ossimBilinearProjection(theImageClipRect.ul(),
                                                    theImageClipRect.ur(),
                                                    theImageClipRect.lr(),
                                                    theImageClipRect.ll(),
                                                    ulg,
                                                    urg,
                                                    lrg,
                                                    llg);

      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::updateModel(): returning..." << std::endl;
   }
   catch(...)
   {
      setErrorStatus(ossimErrorCodes::OSSIM_ERROR);
   }
}

void ossimTheosModel::initAdjustableParameters()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::initAdjustableParameters(): entering..." << std::endl;

   //---
   // Allocate storage for adjustables and assign their names and units
   // strings.
   //---
   resizeAdjustableParameterArray(7);
   ossim_uint32 numParams = getNumberOfAdjustableParameters();

   //---
   // Initialize base-class adjustable parameter array:
   //---
   for (ossim_uint32 i=0; i<numParams; ++i)
   {
      setAdjustableParameter(i, 0.0);
      setParameterDescription(i, PARAM_NAMES[i]);
      setParameterUnit(i,PARAM_UNITS[i]);
      setParameterSigma(i, SIGMA[i]);
   }

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::initAdjustableParameters(): returning..." << std::endl;
}

void ossimTheosModel::loadSupportData()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "ossimTheosModel::loadSupportData(): entering..." << std::endl;

   //---
   // Check for good support data:
   //---
   if (!theSupportData)
   {
      setErrorStatus();
      ossimNotify(ossimNotifyLevel_FATAL) << "FATAL ossimTheosModel::loadSupportData(): Null TheosDimapSupportData pointer passed to"
                                          << " constructor! Aborting..." << std::endl;
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::loadSupportData(): returning..." << std::endl;
      return;
   }

   if (theSupportData->getErrorStatus() != ossimErrorCodes::OSSIM_OK)
   {
      setErrorStatus();
      ossimNotify(ossimNotifyLevel_FATAL) << "FATAL ossimTheosModel::loadSupportData(): Bad TheosDimapSupportData detected. Aborting..."
                                          << std::endl;
      if (traceExec()) ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::loadSupportData(): returning..." << std::endl;
      return;
   }

   //---
   // Initialize some member variables from the support data:
   //---
   theSensorID     = theSupportData->getSensorID();
   theImageID      = theSupportData->getImageID();
   theMetaDataFile = theSupportData->getMetadataFile();

   // Center of frame, sub image if we have one.
   theSupportData->getRefGroundPoint(theRefGndPt);

   theSupportData->getSunAzimuth(theIllumAzimuth);
   theSupportData->getSunElevation(theIllumElevation);
   ossimDpt sz;
   theSupportData->getImageSize(sz);
   theImageSize = sz;
   theSupportData->getRefLineTime(theRefImagingTime);
   theSupportData->getRefLineTimeLine(theRefImagingTimeLine);

   theSupportData->getLineSamplingPeriod(theLineSamplingPeriod);
   theSupportData->getSubImageOffset(theTheosSubImageOffset);

   //---
   // We make this zero base as the base ossimSensorModel does not know about
   // any sub image we have.
   //---
   theSupportData->getImageRect(theImageClipRect);
   theSupportData->getRefImagePoint(theRefImgPt);

   ossimGpt p1;
   ossimGpt p2;
   ossimGpt p3;
   ossimGpt p4;


   // I need to find the nominal scale of the theos dataset

   //---
   // Position error is a function of whether star tracker information was
   // available:
   //---
   if (theSupportData->isStarTrackerUsed())
   {
      thePositionError = 50.0;
   }
   else
   {
      thePositionError = 200.0; // meters
   }
   updateModel();
   lineSampleToWorld(theImageClipRect.ul(), p1);
   lineSampleToWorld(theImageClipRect.ur(), p2);
   lineSampleToWorld(theImageClipRect.lr(), p3);
   lineSampleToWorld(theImageClipRect.ll(), p4);

//    theSupportData->getUlCorner(p1);
//    theSupportData->getUrCorner(p2);
//    theSupportData->getLrCorner(p3);
//    theSupportData->getLlCorner(p4);

   ossimDpt v[4]; // temporarily holds vertices for ground polygon
   v[0] = p1;
   v[1] = p2;
   v[2] = p3;
   v[3] = p4;
   theBoundGndPolygon = ossimPolygon(4, v);


   ossimGpt cgpt, hgpt, vgpt;
   // ossimEcefPoint hVector, vVector;
   ossimDpt midpt = theImageClipRect.midPoint();

   lineSampleToWorld(midpt, cgpt);
   lineSampleToWorld(midpt + ossimDpt(1,0), hgpt);
   lineSampleToWorld(midpt + ossimDpt(0,1), vgpt);

   theGSD     = ossimDpt((ossimEcefPoint(cgpt) - ossimEcefPoint(hgpt)).magnitude(),
			 (ossimEcefPoint(cgpt) - ossimEcefPoint(vgpt)).magnitude());

   theMeanGSD = (theGSD.x+theGSD.y)/2.0;

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::loadSupportData(): returning..." << std::endl;
}

ossimObject* ossimTheosModel::dup() const
{
   return new ossimTheosModel(*this);
}

std::ostream& ossimTheosModel::print(std::ostream& out) const
{
   // Capture stream flags since we are going to mess with them.
   std::ios_base::fmtflags f = out.flags();

   out << "\nDump of ossimTheosModel at address " << (hex) << this
       << (dec)
       << "\n------------------------------------------------"
       << "\n  theImageID            = " << theImageID
       << "\n  theMetadataFile       = " << theMetaDataFile
       << "\n  theIllumAzimuth       = " << theIllumAzimuth
       << "\n  theIllumElevation     = " << theIllumElevation
       << "\n  thePositionError      = " << thePositionError
       << "\n  theImageSize          = " << theImageSize
       << "\n  theRefGndPt           = " << theRefGndPt
       << "\n  theRefImgPt           = " << theRefImgPt
       << "\n  theRefImagingTime     = " << theRefImagingTime
       << "\n  theRefImagingTimeLine = " << theRefImagingTimeLine
       << "\n  theLineSamplingPeriod = " << theLineSamplingPeriod
       << "\n  theRollOffset         = " << theRollOffset
       << "\n  thePitchOffset        = " << thePitchOffset
       << "\n  theYawOffset          = " << theYawOffset
       << "\n  theRollRate           = " << theRollRate
       << "\n  thePitchRate          = " << thePitchRate
       << "\n  theYawRate            = " << theYawRate
       << "\n  theFocalLenOffset     = " << theFocalLenOffset
       << "\n------------------------------------------------"
       << "\n  " << endl;

   // Set the flags back.
   out.flags(f);

   return ossimSensorModel::print(out);
}

bool ossimTheosModel::saveState(ossimKeywordlist& kwl,
                          const char* prefix) const
{
  if(theSupportData.valid())
  {
     ossimString supportPrefix = ossimString(prefix) + "support_data.";
     theSupportData->saveState(kwl, supportPrefix);
  }
  else
  {
     return false;
  }

   return ossimSensorModel::saveState(kwl, prefix);
}

bool ossimTheosModel::loadState(const ossimKeywordlist& kwl,
                                const char* prefix)
{
   ossimString supportPrefix = ossimString(prefix) + "support_data.";

   if(!theSupportData)
   {
      theSupportData = new ossimTheosDimapSupportData;
   }

   if(theSupportData->loadState(kwl, supportPrefix))
   {
      if(!ossimSensorModel::loadState(kwl, prefix))
      {
         return false;
      }
   }
   else
   {
      return false;
   }

   loadSupportData();
   updateModel();

   return (getErrorStatus()==ossimErrorCodes::OSSIM_OK);
}

void ossimTheosModel::imagingRay(const ossimDpt& image_point,
                                 ossimEcefRay&   image_ray) const
{
   bool runtime_dbflag = 0;
   NEWMAT::Matrix satToOrbit;
   ossimDpt iPt = image_point;
   iPt.samp += theTheosSubImageOffset.samp;
   iPt.line += theTheosSubImageOffset.line;

   //
   // 1. Establish time of line imaging:
   //
   double t_line = theRefImagingTime +
                   theLineSamplingPeriod*(iPt.line - theRefImagingTimeLine);
   if (traceDebug() || runtime_dbflag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG TheosModel::imagingRay():------------ BEGIN DEBUG PASS ---------------" << std::endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG TheosModel::imagingRay(): t_line = " << t_line << std::endl;
   }

   //
   // 2. Interpolate ephemeris position and velocity (in ECF):
   //
   ossimEcefPoint  tempEcefPoint;
   ossimEcefPoint  P_ecf;
   theSupportData->getPositionEcf(t_line, P_ecf);
   theSupportData->getVelocityEcf(t_line, tempEcefPoint);
   ossimEcefVector V_ecf(tempEcefPoint.x(),
                         tempEcefPoint.y(),
                         tempEcefPoint.z());
   if (traceDebug() || runtime_dbflag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "DEBUG:\n\tP_ecf = " << P_ecf
         << "\n\t V_ecf = " << V_ecf << std::endl;
   }

   //
   // 3. Establish the look direction in Vehicle LSR space (S_sat).
   //    ANGLES IN RADIANS
   //
   ossim_float64 Psi_x;
   ossim_float64 Psi_y;
    theSupportData->getPixelLookAngleX(iPt.samp, Psi_x);
	theSupportData->getPixelLookAngleY(iPt.samp, Psi_y);
	//ossim_float64 tmp = Psi_x;
	//Psi_x = Psi_y;
	//Psi_y = tmp;
	Psi_x = Psi_x * 3.14159265358979 / 180.0;
	Psi_y = Psi_y * 3.14159265358979 / 180.0;
    if (traceDebug() || runtime_dbflag)
    {
       ossimNotify(ossimNotifyLevel_DEBUG)
          << "DEBUG:\n\t Psi_x = " << Psi_x
          << "\n\t Psi_y = " << Psi_y << endl;
    }

    ossimColumnVector3d u_sat (-tan(Psi_y), tan(Psi_x), -(1.0 + theFocalLenOffset));
    if (traceDebug() || runtime_dbflag)
    {
       ossimNotify(ossimNotifyLevel_DEBUG)
          << "DEBUG \n\t u_sat = " << u_sat << endl;
    }

   //
   // 4. Transform vehicle LSR space look direction vector to orbital LSR space
   //    (S_orb):
   //
    computeSatToOrbRotation(satToOrbit, t_line);

    ossimColumnVector3d u_orb = (satToOrbit*u_sat).unit();
    if (traceDebug() || runtime_dbflag)
    {
       ossimNotify(ossimNotifyLevel_DEBUG)
          << "DEBUG:\n\t theSatToOrbRotation = " << satToOrbit
          << "\n\t u_orb = " << u_orb << endl;
    }

   //
   // 5. Transform orbital LSR space look direction vector to ECF.
   //
   //   a. S_orb space Z-axis (Z_orb) is || to the ECF radial vector (P_ecf),
   //   b. X_orb axis is computed as cross-product between velocity and radial,
   //   c. Y_orb completes the orthogonal S_orb coordinate system.
   //
    ossimColumnVector3d Z_orb (P_ecf.x(),
                               P_ecf.y(),
                               P_ecf.z());
    Z_orb = Z_orb.unit();

    ossimColumnVector3d X_orb = ossimColumnVector3d(V_ecf.x(),
                                                    V_ecf.y(),
                                                    V_ecf.z()).cross(Z_orb).unit();
    ossimColumnVector3d Y_orb = Z_orb.cross(X_orb);

    NEWMAT::Matrix orbToEcfRotation = NEWMAT::Matrix(3, 3);
    orbToEcfRotation << X_orb[0] << Y_orb[0] << Z_orb[0]
                        << X_orb[1] << Y_orb[1] << Z_orb[1]
                        << X_orb[2] << Y_orb[2] << Z_orb[2];


   ossimColumnVector3d u_ecf  = (orbToEcfRotation*u_orb);
    if (traceDebug() || runtime_dbflag)
    {
       ossimNotify(ossimNotifyLevel_DEBUG)
          << "DEBUG:\n\t orbToEcfRotation = " << orbToEcfRotation
          << "\n\t u_ecf = " << u_ecf << endl;
    }

   //
   // Establish the imaging ray given direction and origin:
   //
    image_ray = ossimEcefRay(P_ecf, ossimEcefVector(u_ecf[0], u_ecf[1], u_ecf[2]));

    if (traceExec())
    {
       ossimNotify(ossimNotifyLevel_DEBUG)
          << "DEBUG TheosModel::imagingRay(): returning..." << std::endl;
    }
}

void ossimTheosModel::lineSampleHeightToWorld(const ossimDpt& image_point,
                                              const ossim_float64& heightEllipsoid,
                                              ossimGpt& worldPoint) const
{
//   if (!insideImage(image_point))
   if ( !theImageClipRect.pointWithin(image_point, 1.0-FLT_EPSILON) )   
   {
      if(theSeedFunction.valid())
      {
         theSeedFunction->lineSampleToWorld(image_point, worldPoint);
      }
      else
      {
         worldPoint = extrapolate(image_point, heightEllipsoid);
      }
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::lineSampleHeightToWorld(): returning..." << std::endl;
      return;
   }
   //***
   // First establish imaging ray from image point:
   //***
   ossimEcefRay imaging_ray;
   imagingRay(image_point, imaging_ray);
   //ossimEcefPoint Pecf (imaging_ray.intersectAboveEarthEllipsoid(heightEllipsoid,m_proj->getDatum()));
   //worldPoint = ossimGpt(Pecf, aDatumWant);
   if(m_proj) {
	   ossimEcefPoint Pecf (imaging_ray.intersectAboveEarthEllipsoid(heightEllipsoid,m_proj->getDatum()));
	   worldPoint = ossimGpt(Pecf,m_proj->getDatum());
   }
   else
   {
	   ossimEcefPoint Pecf (imaging_ray.intersectAboveEarthEllipsoid(heightEllipsoid));
	   worldPoint = ossimGpt(Pecf);

   }
}

// ossimDpt ossimTheosModel::extrapolate (const ossimGpt& gp) const
// {
//     ossimDpt temp;

//     temp.makeNan();

//     return temp;

//   ossimDpt tempGpt = gp;
//   ossimDpt dest;
//   theGroundToImageMap.map(tempGpt, dest);

//  return dest;

// }

// ossimGpt ossimTheosModel::extrapolate (const ossimDpt& ip,
// 				       const double& height) const
// {
//   return ossimGpt(ossim::nan(), ossim::nan(), ossim::nan(), 0);

//    ossimDpt dest;

//    theImageToGroundMap.map(ip, dest);


//    return ossimGpt(dest.lat, dest.lon, ossim::nan(), origin().datum());
// }

bool
ossimTheosModel::setupOptimizer(const ossimString& init_file)
{
   ossimFilename theosTest = init_file;
   ossimFilename geomFile = init_file;
   geomFile = geomFile.setExtension("geom");
   bool tryKwl = false;

   if(!theosTest.exists())
   {
      theosTest = geomFile.path();
      theosTest = theosTest.dirCat(ossimFilename("METADATA.DIM"));
      if(theosTest.exists() == false)
      {
         theosTest = geomFile.path();
         theosTest = theosTest.dirCat(ossimFilename("metadata.dim"));
      }
   }
   if(theosTest.exists())
   {
      ossimRefPtr<ossimTheosDimapSupportData> meta = new ossimTheosDimapSupportData;
      if(meta->loadXmlFile(theosTest))
      {
         initFromMetadata(meta.get());
         if (getErrorStatus())
         {
            tryKwl = true;
            meta=0;
         }
         else
         {
            return true;
         }
      }
      else
      {
         meta=0;
         tryKwl = true;
      }
   }
   if(tryKwl)
   {
      ossimKeywordlist kwl;
      if(kwl.addFile(init_file.c_str()))
      {
         return loadState(kwl);
      }
   }
   return false;
}

bool
ossimTheosModel::initFromMetadata(ossimTheosDimapSupportData* sd)
{
   // init parms
   theSupportData        = sd;
   theMetaDataFile       = "NOT ASSIGNED";
   theIllumAzimuth       = 0.0;
   theIllumElevation     = 0.0;
   thePositionError      = 0.0;
   theRefImagingTime     = 0.0;
   theLineSamplingPeriod = 0.0;
//   theSatToOrbRotation   = 0.0; //matrix
//   theOrbToEcfRotation   = 0.0; //matrix
   theRollOffset         = 0.0;
   thePitchOffset        = 0.0;
   theYawOffset          = 0.0;
   theRollRate           = 0.0;
   thePitchRate          = 0.0;
   theYawRate            = 0.0;
   theFocalLenOffset     = 0.0;

   //---
   // Instantiate the support data classes after establishing the filenames:
   //---
   loadSupportData();
   if (getErrorStatus() != ossimErrorCodes::OSSIM_OK)
   {
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimTheosModel::initFromMetadata(dimap_file): returning with error..." << std::endl;
      return false;
   }

   //---
   // initialize remaining data members:
   //---
   initAdjustableParameters();
   updateModel();
   return true;
}
