//*******************************************************************
//
// License:  See top level LICENSE.txt file.
//
// Author:  Oscar Kramer (ossim port by D. Burken)
//
// Description:
//
// Contains definition of class ossimHjModel.
//
//*****************************************************************************
// $Id: ossimHjModel.cpp 19658 2011-05-26 13:16:06Z gpotts $

#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

#include <ossim/radi/ossimHjModel.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/support_data/ossimSpotDimapSupportData.h>
#include <ossim/projection/ossimMapProjection.h>
#include <ossim/base/ossimLsrPoint.h>
#include <ossim/base/ossimEcefRay.h>
#include <ossim/base/ossimLsrRay.h>
#include <ossim/base/ossimLsrSpace.h>
#include <ossim/base/ossimDpt3d.h>
#include <ossim/base/ossimColumnVector3d.h>
#include <ossim/base/ossimNotifyContext.h>
#include <ossim/elevation/ossimElevManager.h>

RTTI_DEF1(ossimHjModel, "ossimHjModel", ossimSensorModel);


//---
// Define Trace flags for use within this file:
//---
#include <ossim/base/ossimTrace.h>
static ossimTrace traceExec  ("ossimHjModel:exec");
static ossimTrace traceDebug ("ossimHjModel:debug");

static const ossim_int32 MODEL_VERSION_NUMBER = 1;

static const char* PARAM_NAMES[] = { "ccd_roll_offset",
                                     "ccd_pitch_offset",
                                     "ccd_yaw_offset",
									 "x_offset",
									 "y_offset",
									 "z_offset",
									 "focal_offset",
									 "ay0_offset",
									 "ay1_offset",
									 "ay2_offset",
									 "ay3_offset",
									 "ay4_offset",
									 "ay5_offset",
									 "ay6_offset",
								     "line_offset"};

static const char* PARAM_UNITS[] = { 
									 "degrees",   // degrees
									 "degrees",   // degrees
									 "degrees",   // degrees,
									 "none",
									 "none",
									 "none",
									 "meter",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "pixel"}; 

static const ossim_float64 SIGMA[] = { 1.0e-4,  // ccd_roll_offset delta degrees
									   1.0e-4,  // ccd_pitch_offset delta degrees
									   1.0e-4,  // ccd_yaw_offset delta degrees
									   1.0e-4,  // x_offset
									   1.0e-4,  // y_offset
									   1.0e-4,  // z_offset
									   1.0e-4,	 // focal_offset
									   1.0e-4,	// ay0_offset
									   1.0e-4,	// ay1_offset
									   1.0e-4,	// ay2_offset
									   1.0e-4,	// ay3_offset
									   1.0e-4,	// ay4_offset
									   1.0e-4,	// ay5_offset
									   1.0e-4,	// ay6_offset
									   1.0		// line_offset
};

ossimHjModel::ossimHjModel()
   :
   ossimSensorModel      (),
   theSupportData        (NULL),
   theMetaDataFile       ("NOT ASSIGNED"),
   theIllumAzimuth       (0.0),
   theIllumElevation     (0.0),
   thePositionError      (0.0),
   theLineSamplingPeriod (0.0),
   theRefImagingTime(0.0),
   theRefImagingTimeLine(0.0),
//   theSatToOrbRotation   (3, 3),
//   theOrbToEcfRotation   (3, 3),
   //theRollOffset         (0.0),
   //thePitchOffset        (0.0),
   //theYawOffset          (0.0),
   theCcdRollOffset        (0.0),
   theCcdPitchOffset	   (0.0),
   theCcdYawOffset	       (0.0),
   theXOffset            (0.0),
   theYOffset            (0.0),
   theZOffset            (0.0),
   theFocalOffset        (0.0),
   theAY0Offset	         (0.0),
   theAY1Offset	         (0.0),
   theAY2Offset	         (0.0),
   theAY3Offset	         (0.0),
   theAY4Offset	         (0.0),
   theAY5Offset	         (0.0),
   theAY6Offset	         (0.0),
   theLineOffset         (0.0),
   theFocalLen(1.4078),	// meters
   //theFocalLen(1.3831539),
   theCcdStepLen(0.00006507)	// meters
{
   initAdjustableParameters();
}

ossimHjModel::ossimHjModel(ossimQVProcSupportData* sd)
   :
   ossimSensorModel      (),
   theSupportData        (sd),
   theMetaDataFile       ("NOT ASSIGNED"),
   theIllumAzimuth       (0.0),
   theIllumElevation     (0.0),
   thePositionError      (0.0),
   theLineSamplingPeriod (0.0),
   theRefImagingTime(0.0),
   theRefImagingTimeLine(0.0),
//   theSatToOrbRotation   (3, 3),
//   theOrbToEcfRotation   (3, 3),
   //theRollOffset         (0.0),
   //thePitchOffset        (0.0),
   //theYawOffset          (0.0),
   theCcdRollOffset        (0.0),
   theCcdPitchOffset	   (0.0),
   theCcdYawOffset	       (0.0),
   theXOffset            (0.0),
   theYOffset            (0.0),
   theZOffset            (0.0),
   theFocalOffset        (0.0),
   theAY0Offset	         (0.0),
   theAY1Offset	         (0.0),
   theAY2Offset	         (0.0),
   theAY3Offset	         (0.0),
   theAY4Offset	         (0.0),
   theAY5Offset	         (0.0),
   theAY6Offset	         (0.0),
   theLineOffset         (0.0),
   theFocalLen(1.4078),	// meters
   //theFocalLen(1.3831539),
   theCcdStepLen(0.000065)	// meters
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel(desc_file) Constructor: entering..." << std::endl;

   //---
   // Instantiate the support data classes after establishing the filenames:
   //---
   loadSupportData();
   if (getErrorStatus() != ossimErrorCodes::OSSIM_OK)
   {
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel(desc_file) Constructor: returning with error..." << std::endl;
      return;
   }

   //---
   // initialize remaining data members:
   //---
   initAdjustableParameters();
   updateModel();

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel(desc_file) Constructor: returning..." << std::endl;
}

//*****************************************************************************
//  DESTRUCTOR: ~ossimHjModel()
//
//*****************************************************************************
ossimHjModel::~ossimHjModel()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG DESTRUCTOR: ~ossimHjModel(): entering..." << std::endl;

   theSupportData = 0;
   
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG DESTRUCTOR: ~ossimHjModel(): returning..." << std::endl;
}

ossimHjModel::ossimHjModel(const ossimHjModel& rhs)
   :ossimSensorModel(rhs)
{
   if(rhs.theSupportData.valid())
   {
      theSupportData = (ossimQVProcSupportData*)rhs.theSupportData->dup();
   }
   loadSupportData();
   updateModel();
}


void ossimHjModel::getCcdToSatRotation(NEWMAT::Matrix& result)const
{
	result = NEWMAT::Matrix(3,3);
	double roll, pitch, yaw;
	if (theSensorID.upcase().contains("HJ-1A CCD-1"))
	{
		roll = 15.182276 * M_PI / 180.0;
		pitch = 0.304668 * M_PI / 180.0;
		yaw = 0.106528 * M_PI / 180.0;

		//result << 1.0 << 1.4719e-5 << -9.024e-4
		//	<< -1.2218e-5 << 1 << 2.7709e-3
		//	<< 9.0244e-4 << -2.7709e-3 << 1.0;
	}
	else if(theSensorID.upcase().contains("HJ-1A CCD-2"))
	{
		roll = -14.816663 * M_PI / 180.0;
		pitch = 0.552111 * M_PI / 180.0;
		yaw = 0.07 * M_PI / 180.0;
	}
	else if(theSensorID.upcase().contains("HJ-1B CCD-1"))
	{
		roll = 15.145556 * M_PI / 180.0;// +0.0001383701401;
		//pitch = 0.441230 * M_PI / 180.0 -0.0580030436;
		pitch = 0.441230 * M_PI / 180.0;// +0.0580030436;
		//pitch = 0.441230 * M_PI / 180.0 -0.0005800304;
		yaw = 0.120700 * M_PI / 180.0;// -0.05730759;
	}
	else if(theSensorID.upcase().contains("HJ-1B CCD-2"))
	{
		roll = -15.015556 * M_PI / 180.0;
		pitch = 0.362064 * M_PI / 180.0;
		yaw = 0.300700 * M_PI / 180.0;
	}
	//NEWMAT::Matrix m1(3,3);
	//NEWMAT::Matrix m2(3,3);
	//NEWMAT::Matrix m3(3,3);
	//m1 << cos(pitch) << 0.0 << -sin(pitch)
	//	<< 0.0 << 1.0 << 0.0
	//	<< sin(pitch) << 0 << cos(pitch);
	//m2 << 1.0 << 0.0 << 0.0
	//	<< 0.0 << cos(roll) << sin(roll)
	//	<< 0 << -sin(roll) << cos(roll);
	//m3 << cos(yaw) << sin(yaw) << 0.0
	//	<< -sin(yaw) << cos(yaw) << 0.0
	//	<< 0.0 <<0.0 << 1.0;
	//result = m1 * m2 * m3;
	double x = (pitch + theCcdPitchOffset);
	double y = (roll + theCcdRollOffset);
	double z = (yaw + theCcdYawOffset);

	double cp = cos(x);
	double sp = sin(x);
	double cr = cos(y);
	double sr = sin(y);
	double cy = cos(z);
	double sy = sin(z);

	//---
	// Populate rotation matrix:
	//---
	//result << (cr*cy) << (-cr*sy) << (-sr)
	//<< (cp*sy+sp*sr*cy) << (cp*cy-sp*sr*sy) << (sp*cr)
	//<< (-sp*sy+cp*sr*cy) << (-sp*cy-cp*sr*sy) <<  cp*cr;

	result << (cr*cy-sp*sr*sy) << (cr*sy+sp*sr*sy) << (-cp*sr)
		<< (-cp*sy) << (cp*cy) << (sp)
		<< (sr*cy+sp*cr*sy) << (sr*sy-sp*cr*cy) <<  cp*cr;
	//cout<<result;
}

void ossimHjModel::computeSatToOrbRotation(NEWMAT::Matrix& result, ossim_float64 t)const
{
   if (traceExec())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
      << "DEBUG ossimHjModel::computeSatToOrbRotation(): entering..."
      << std::endl;
   }
   //---
   // Linearly interpolate attitudes angles:
   //---
   ossimDpt3d att(0.0, 0.0, 0.0);
   theSupportData->getAttitude(t, att);
   //att.x = -att.x;
   //att.y = -att.y;
   //att.z = -att.z;
   //---
   // Apply the attitude adjustable parameters:
   //---

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
   result = NEWMAT::Matrix(3,3);
   //result << (cr*cy) << (-cr*sy) << (-sr)
   //<< (cp*sy+sp*sr*cy) << (cp*cy-sp*sr*sy) << (sp*cr)
   //<< (-sp*sy+cp*sr*cy) << (-sp*cy-cp*sr*sy) <<  cp*cr;

   result << (cr*cy-sp*sr*sy) << (cr*sy+sp*sr*sy) << (-cp*sr)
	   << (-cp*sy) << (cp*cy) << (sp)
	   << (sr*cy+sp*cr*sy) << (sr*sy-sp*cr*cy) <<  cp*cr;

   NEWMAT::Matrix hj2spot(3, 3);
   hj2spot << 0.0 << 1.0 << 0.0
	   << 1.0 << 0.0 << 0.0
	   << 0.0 << 0.0 << 1.0;

   //result = result * hj2spot;

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::computeSatToOrbRotation(): returning..." << std::endl;
}

#if 0
//*****************************************************************************
//  METHOD
//*****************************************************************************
void ossimHjModel::computeSatToOrbRotation(ossim_float64 t)const
{
   if (traceExec())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "DEBUG ossimHjModel::computeSatToOrbRotation(): entering..."
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


    if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::computeSatToOrbRotation(): returning..." << std::endl;
}
#endif


ossim_float64 ossimHjModel::computeLookAngleY(double samp, double ccdOffset/* = 0.0*/) const
{
	ossim_float64 look_angle_Y;
	double start_angle;
	double midCCD = theImageSize.samp * 0.5 + ccdOffset;
	//double midCCD = ccdOffset;
	if (theSensorID.upcase().contains("CCD-1"))
	{
		start_angle = 0.2618;	// radians
		//look_angle_X = atan(tan(start_angle+start_offset)
		//	-(theImageSize.samp-samp)*theCcdStepLen/(theFocalLen*(1+theFocalLenOffset)));
	}
	else if(theSensorID.upcase().contains("CCD-2"))
	{
		start_angle = -0.2618;	// radians
		//look_angle_X = atan(tan(start_angle+start_offset)
		//	+samp*theCcdStepLen/(theFocalLen*(1+theFocalLenOffset)));
	}
	start_angle = 0.0;
	look_angle_Y = start_angle + atan((samp - midCCD)*theCcdStepLen/(theFocalLen+theFocalOffset));

	return look_angle_Y;
}

ossim_float64 ossimHjModel::computePhiX(double samp) const
{
	double ax0;
	if (theSensorID.upcase().contains("HJ-1A CCD-1"))
	{
		ax0 = 0.0;
	}
	else if(theSensorID.upcase().contains("HJ-1A CCD-2"))
	{
		ax0 = 0.0;
	}
	else if(theSensorID.upcase().contains("HJ-1B CCD-1"))
	{
		ax0 = 0.0;// + 0.0948961;
	}
	else if(theSensorID.upcase().contains("HJ-1B CCD-2"))
	{
		ax0 = 0.0;
	}
	//ax0 += theAX0Offset;
	return ax0;
	//double ax0 = 0.0;
	//double ax1 = 0.0;
	//double ax2 = 0.0;
	//double ax3 = 0.0;

	//ax0 += theAX0Offset;
	//ax1 += theAX1Offset;
	//ax2 += theAX2Offset;
	//ax3 += theAX3Offset;

	//ossim_float64 phiX = ax0 + ax1 * samp
	//	+ ax2 * samp * samp + ax3 * samp * samp *samp;

	//return phiX;
}

ossim_float64 ossimHjModel::computePhiY(double samp) const
{
	double midCCD = theImageSize.samp * 0.5;
	double k = (theCcdStepLen)/(theFocalLen+theFocalOffset) * 1.0e0;
	double x = (samp - midCCD) * 1.0e-0;
	//double kx = k * x;
	double kx = x / midCCD;
	//double ay0 = -atan(midCCD * theCcdStepLen / theFocalLen);
	double ay0 = 0.0;
	double ay1 = 0.0;
	double ay2 = 0.0;
	double ay3 = 0.0;
	double ay4 = 0.0;
	double ay5 = 0.0;
	double ay6 = 0.0;
	//double ay0 = k;
	//double ay1 = -k*k*k/3.0;
	//double ay2 = k*k*k*k*k/5.0;
	//double ay3 = -k*k*k*k*k*k*k/7.0;

	if (theSensorID.upcase().contains("HJ-1A CCD-1"))
	{
	}
	else if(theSensorID.upcase().contains("HJ-1A CCD-2"))
	{
	}
	else if(theSensorID.upcase().contains("HJ-1B CCD-1"))
	{
		//ay0 = 0.0 + 0.00314632145;
		//ay1 = 1.0 -0.0033606787;
		//ay2 = 0.0 + 0.0049293;
		//ay3 = -1.0/3.0 + 0.393465447;
		//ay4 = 0.0 -0.011753576;
		ay0 = 0.0;
		ay1 = 0.0;
		ay2 = 0.0;
		ay3 = 0.0;
		ay4 = 0.0;
	}
	else if(theSensorID.upcase().contains("HJ-1B CCD-2"))
	{
	}

	ay0 += theAY0Offset;
	ay1 += theAY1Offset;
	ay2 += theAY2Offset;
	ay3 += theAY3Offset;
	ay4 += theAY4Offset;
	ay5 += theAY5Offset;
	ay6 += theAY6Offset;

	//ossim_float64 phiY = kx 
	//	+ ay0 * kx
	//	+ ay1 * kx * kx * kx
	//	+ ay2 * kx * kx * kx * kx * kx
	//	+ ay3 * kx * kx * kx * kx * kx * kx * kx
	//	+ ay4 * kx * kx * kx * kx * kx * kx * kx * kx * kx;
	ossim_float64 phiY = kx
		+ ay0
		+ ay1 * kx
		+ ay2 * kx * kx
		+ ay3 * kx * kx * kx
		+ ay4 * kx * kx * kx * kx
		+ ay5 * kx * kx * kx * kx * kx
		+ ay6 * kx * kx * kx * kx * kx * kx;
	//ossim_float64 phiY = ay0 * x
	//	+ ay1 * x * x * x
	//	+ ay2 * x * x * x * x * x
	//	+ ay3 * x * x * x * x * x * x * x;
	phiY = phiY * midCCD * theCcdStepLen;
	return phiY;
}

//*****************************************************************************
// PUBLIC METHOD: ossimHjModel::updateModel()
//
//  Updates the model parameters given the normalized adjustable parameter
//  array.
//
//*****************************************************************************
void ossimHjModel::updateModel()
{
   clearErrorStatus();

   try
   {
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::updateModel(): entering..." << std::endl;

      if(getNumberOfAdjustableParameters() < 1)
      {
         theCcdRollOffset       = 0;
         theCcdPitchOffset      = 0;
         theCcdYawOffset        = 0;
		 theXOffset   = 0;
		 theYOffset   = 0;
		 theZOffset   = 0;
		 theFocalOffset    = 0;
		 theAY0Offset = 0;
		 theAY1Offset	= 0;
		 theAY2Offset = 0;
		 theAY3Offset = 0;
		 theAY4Offset = 0;
		 theAY5Offset = 0;
		 theAY6Offset = 0;
		 theLineOffset = 0;
      }
      else
      {
		  int pos = 0;
		  theCcdRollOffset       = computeParameterOffset(pos++);
		  theCcdPitchOffset      = computeParameterOffset(pos++);
		  theCcdYawOffset        = computeParameterOffset(pos++);
		  theXOffset             = computeParameterOffset(pos++);
		  theYOffset             = computeParameterOffset(pos++);
		  theZOffset             = computeParameterOffset(pos++);
		 theFocalOffset      = computeParameterOffset(pos++);
		 theAY0Offset = computeParameterOffset(pos++);
		 theAY1Offset	   = computeParameterOffset(pos++);
		 theAY2Offset = computeParameterOffset(pos++);
		 theAY3Offset = computeParameterOffset(pos++);
		 theAY4Offset = computeParameterOffset(pos++);
		 theAY5Offset      = computeParameterOffset(pos++);
		 theAY6Offset      = computeParameterOffset(pos++);
		 theLineOffset      = computeParameterOffset(pos++);
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

      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::updateModel(): returning..." << std::endl;
   }
   catch(...)
   {
      setErrorStatus(ossimErrorCodes::OSSIM_ERROR);
   }
}

void ossimHjModel::initAdjustableParameters()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::initAdjustableParameters(): entering..." << std::endl;

   //---
   // Allocate storage for adjustables and assign their names and units
   // strings.
   //---
   resizeAdjustableParameterArray(NUM_ADJUSTABLE_PARAMS);
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

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::initAdjustableParameters(): returning..." << std::endl;
}

void ossimHjModel::loadSupportData()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "ossimHjModel::loadSupportData(): entering..." << std::endl;

   //---
   // Check for good support data:
   //---
   if (!theSupportData)
   {
      setErrorStatus();
      ossimNotify(ossimNotifyLevel_FATAL) << "FATAL ossimHjModel::loadSupportData(): Null SpotDimapSupportData pointer passed to"
                                          << " constructor! Aborting..." << std::endl;
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::loadSupportData(): returning..." << std::endl;
      return;
   }

   if (theSupportData->getErrorStatus() != ossimErrorCodes::OSSIM_OK)
   {
      setErrorStatus();
      ossimNotify(ossimNotifyLevel_FATAL) << "FATAL ossimHjModel::loadSupportData(): Bad SpotDimapSupportData detected. Aborting..."
                                          << std::endl;
      if (traceExec()) ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::loadSupportData(): returning..." << std::endl;
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

   theSupportData->getLineSamplingPeriod(theLineSamplingPeriod);
   theSupportData->getRefImagingTime(theRefImagingTime);
   theSupportData->getRefImagingTimeLine(theRefImagingTimeLine);
   theSupportData->getSubImageOffset(theSpotSubImageOffset);
   
   //---
   // We make this zero base as the base ossimSensorModel does not know about
   // any sub image we have.
   //---
   theSupportData->getImageRect(theImageClipRect);
   //theSupportData->getRefImagePoint(theRefImgPt);
   theRefImgPt = theImageClipRect.midPoint();

   ossimGpt p1;
   ossimGpt p2;
   ossimGpt p3;
   ossimGpt p4;

   // I need to find the nominal scale of the spot 5 dataset

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

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::loadSupportData(): returning..." << std::endl;
}

ossimObject* ossimHjModel::dup() const
{
   return new ossimHjModel(*this);
}

std::ostream& ossimHjModel::print(std::ostream& out) const
{
   // Capture stream flags since we are going to mess with them.
   std::ios_base::fmtflags f = out.flags();

   out << "\nDump of ossimHjModel at address " << (hex) << this
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
	   << "\n  theLineSamplingPeriod = " << theLineSamplingPeriod
	   << "\n  theRefImagingTime = " << theRefImagingTime
	   << "\n  theRefImagingTimeLine = " << theRefImagingTimeLine
    //   << "\n  theRollOffset         = " << theRollOffset
    //   << "\n  thePitchOffset        = " << thePitchOffset
	//<< "\n  theYawOffset          = " << theYawOffset
	<< "\n  theCcdRollOffset           = " << theCcdRollOffset
	<< "\n  theCcdPitchOffset          = " << theCcdPitchOffset
	<< "\n  theCcdYawOffset            = " << theCcdYawOffset
	<< "\n  theXOffset            = " << theXOffset
	<< "\n  theYOffset            = " << theYOffset
	<< "\n  theZOffset            = " << theZOffset
	   << "\n  theFocalOffset        = " << theFocalOffset
	   << "\n  theAY0Offset     = " << theAY0Offset
	   << "\n  theAY1Offset     = " << theAY1Offset
	   << "\n  theAY2Offset     = " << theAY2Offset
	   << "\n  theAY3Offset     = " << theAY3Offset
	   << "\n  theAY4Offset     = " << theAY4Offset
	   << "\n  theAY5Offset     = " << theAY5Offset
	   << "\n  theAY6Offset     = " << theAY6Offset
	   << "\n  theLineOffset     = " << theLineOffset 
       << "\n------------------------------------------------"
       << "\n  " << endl;

   // Set the flags back.
   out.flags(f);

   return ossimSensorModel::print(out);
}

bool ossimHjModel::saveState(ossimKeywordlist& kwl,
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

bool ossimHjModel::loadState(const ossimKeywordlist& kwl,
                                const char* prefix)
{
   ossimString supportPrefix = ossimString(prefix) + "support_data.";

   if(!theSupportData)
   {
      theSupportData = new ossimQVProcSupportData;
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

void ossimHjModel::imagingRay(const ossimDpt& image_point,
                                 ossimEcefRay&   image_ray) const
{
   bool runtime_dbflag = 0;
   NEWMAT::Matrix satToOrbit;
   ossimDpt iPt = image_point;
   //iPt.samp += theSpotSubImageOffset.samp;
   //iPt.line += theSpotSubImageOffset.line;

   //
   // 1. Establish time of line imaging:
   //
   //double t_line = theRefImagingTime +
   //                theLineSamplingPeriod*(iPt.line - theRefImagingTimeLine);
   //double line_mean = theImageClipRect.height()*0.5;
   //double scaled_line = (iPt.line - line_mean) / line_mean;

   //double a0 = 0.0;
   //double a1 = 0.0;
   //double a2 = 0.0;
   //double a3 = 0.0;
   //double a4 = 0.0;
   //if (theSensorID.upcase().contains("HJ-1A CCD-1"))
   //{
   //}
   //else if(theSensorID.upcase().contains("HJ-1A CCD-2"))
   //{
   //}
   //else if(theSensorID.upcase().contains("HJ-1B CCD-1"))
   //{
	  // //a0 += -1568.205-37.8783;
	  // //a1 += 566.6828+524.572;
	  // //a2 += -2221.218-1506.3;
	  // //a3 += 3041.41+1871.32;
	  // //a4 += -1375.749-832.485;
   //}
   //else if(theSensorID.upcase().contains("HJ-1B CCD-2"))
   //{
   //}
   //a0 += theLineA0Offset;
   //a1 += theLineA1Offset;
   //a2 += theLineA2Offset;
   //a3 += theLineA3Offset;
   //a4 += theLineA4Offset;
   ////double t_line = iPt.line + theLineOffset;
   //double t_line = iPt.line
	  // + a0 
	  // + a1 * scaled_line
	  // + a2 * scaled_line * scaled_line
	  // + a3 * scaled_line * scaled_line * scaled_line
	  // + a4 * scaled_line * scaled_line * scaled_line * scaled_line;
   double t_line = iPt.line + theLineOffset;
   if (traceDebug() || runtime_dbflag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::imagingRay():------------ BEGIN DEBUG PASS ---------------" << std::endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::imagingRay(): t_line = " << t_line << std::endl;
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

   double omega = 7.292115e-5;
   //ossimColumnVector3d vel_earth_rotation(-omega * P_ecf.y(), omega * P_ecf.x(), 0.0);
   ossimColumnVector3d vel_earth_rotation(0.0, 0.0, omega);
   //V_ecf = V_ecf - vel_earth_rotation.cross(ossimColumnVector3d(P_ecf.x(), P_ecf.y(), P_ecf.z()));
   //std::vector<double> vel_earth_rotation(3, 0.0);
   //vel_earth_rotation[0] = omega *  P_ecf.y();
   //vel_earth_rotation[1] = omega * P_ecf.x();
   //ossimEcefVector V_ecf(tempEcefPoint.x() + vel_earth_rotation[0],
	  // tempEcefPoint.y() + vel_earth_rotation[1],
	  // tempEcefPoint.z() + vel_earth_rotation[2]);
   if (traceDebug() || runtime_dbflag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "DEBUG:\n\tP_ecf = " << P_ecf
         << "\n\t V_ecf = " << V_ecf << std::endl;
   }


   P_ecf = ossimEcefPoint(P_ecf.x(),
	   P_ecf.y(),
	   P_ecf.z());

   //
   // 3. Establish the look direction in Vehicle LSR space (S_sat).
   //    ANGLES IN RADIANS
   //
    //ossim_float64 Psi_x;
    //theSupportData->getPixelLookAngleX(iPt.samp, Psi_x);
    //ossim_float64 Psi_y;
    //theSupportData->getPixelLookAngleY(iPt.samp, Psi_y);
    //if (traceDebug() || runtime_dbflag)
    //{
    //   ossimNotify(ossimNotifyLevel_DEBUG)
    //      << "DEBUG:\n\t Psi_x = " << Psi_x
    //      << "\n\t Psi_y = " << Psi_y << endl;
    //}

    //ossimColumnVector3d u_sat (-tan(Psi_y), tan(Psi_x), -(1.0 + theFocalLenOffset));
    //if (traceDebug() || runtime_dbflag)
    //{
    //   ossimNotify(ossimNotifyLevel_DEBUG)
    //      << "DEBUG \n\t u_sat = " << u_sat << endl;
    //}
    ossim_float64 phiX = computePhiX(iPt.samp);
	ossim_float64 phiY = computePhiY(iPt.samp);
	//ossimColumnVector3d u_sat(-tan(phiY), tan(phiX), -1.0);
	//ossimColumnVector3d u_sat(phiY, phiX, 1.0);
	ossimColumnVector3d u_sat(-phiY, phiX, (theFocalLen + theFocalOffset));

	// 
	NEWMAT::Matrix ccdToSat;
	getCcdToSatRotation(ccdToSat);

	ossimColumnVector3d u_camera = ccdToSat*u_sat;
	ossimColumnVector3d camera_offset(theXOffset, theYOffset, theZOffset);

   //
   // 4. Transform vehicle LSR space look direction vector to orbital LSR space
   //    (S_orb):
   //
    computeSatToOrbRotation(satToOrbit, t_line);

    ossimColumnVector3d u_orb = (satToOrbit*(u_camera+camera_offset)).unit();
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
	//ossimColumnVector3d vel_earth_rotation(-omega * P_ecf.y(), omega * P_ecf.x(), 0.0);
	//V_ecf = V_ecf - vel_earth_rotation.cross(ossimColumnVector3d(P_ecf.x(), P_ecf.y(), P_ecf.z()));
    ossimColumnVector3d Z_orb (P_ecf.x(),
                               P_ecf.y(),
                               P_ecf.z());
 //   Z_orb = Z_orb.unit();

 //   ossimColumnVector3d X_orb = ossimColumnVector3d(V_ecf.x(),
 //                                                   V_ecf.y(),
	//												V_ecf.z()).cross(Z_orb).unit();
	//ossimColumnVector3d Y_orb = Z_orb.cross(X_orb);
	Z_orb = -Z_orb.unit();
	ossimColumnVector3d X_orb = Z_orb.cross(ossimColumnVector3d(V_ecf.x(),
		V_ecf.y(),
		V_ecf.z())).unit();
	ossimColumnVector3d Y_orb = X_orb.cross(Z_orb);

    NEWMAT::Matrix orbToEcfRotation = NEWMAT::Matrix(3, 3);
    orbToEcfRotation << X_orb[0] << Y_orb[0] << Z_orb[0]
                        << X_orb[1] << Y_orb[1] << Z_orb[1]
                        << X_orb[2] << Y_orb[2] << Z_orb[2];


	double a = calcGAST(theRefImagingTime);
	double sa = sin(a);
	double ca = cos(a);
	
	NEWMAT::Matrix j2000ToWGS84 = NEWMAT::Matrix(3, 3);
	j2000ToWGS84 << ca << sa << 0.0
	<< -sa  << ca << 0.0
	<< 0.0 << 0.0 << 1;
	
	//ossimColumnVector3d u_ecf  = (j2000ToWGS84*u_orb).unit();

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
          << "DEBUG Spot5Model::imagingRay(): returning..." << std::endl;
    }
}

double ossimHjModel::calcGAST(double t) const
{
	// calculate Greenwhich Apparent Sidereal Time
	// t is the UTC time (seconds) counting from 2004.1.1 0:0:0

	// calculate the days from 2000.1.1 12:0:0 (UT1 or J2000)
	//double d = t / 86400.0 + 1461.5;
	double d = t / 86400.0 + 1461.5;
	// Compute the number of centuries since J2000
	double T = d/36525.0;

	// THETAm is the mean siderial time in degrees
	// Calculate GMST in hours (0h to 24h) ... then convert to degrees
	double THETAm = 100.46061837 + 36000.770053608*T + 0.000387933*T*T -T*T*T /38710000.0;

	//Mean obliquity of the ecliptic (EPSILONm)
	// see http://www.cdeagle.com/ccnum/pdf/demogast.pdf equation 3
	// also see Vallado, Fundamentals of Astrodynamics and Applications, second edition.
	// pg. 214 EQ 3-53
	double EPSILONm = 23.439291-0.0130111*T - 1.64E-07*T*T + 5.04E-07*T*T*T;

	// Nutations in obliquity and longitude (degrees)
	// see http://www.cdeagle.com/ccnum/pdf/demogast.pdf equation 4
	double L = 280.4665 + 36000.7698*T;
	double dL = 218.3165 + 481267.8813*T;
	double OMEGA = 125.04452 - 1934.136261*T;

	// Calculate nutations using the following two equations:
	// see http://www.cdeagle.com/ccnum/pdf/demogast.pdf equation 5
	double dPSI = -17.20*ossim::sind(OMEGA) - 1.32*ossim::sind(2.0*L) - 0.23*ossim::sind(2.0*dL) + .21*ossim::sind(2.0*OMEGA);
	double dEPSILON = 9.20*ossim::cosd(OMEGA) + 0.57*ossim::cosd(2.0*L) + 0.10*ossim::cosd(2.0*dL) - 0.09*ossim::cosd(2.0*OMEGA);

	// Convert the units from arc-seconds to degrees
	dPSI = dPSI/3600.0;
	dEPSILON = dEPSILON/3600.0;

	// (GAST) Greenwhich apparent sidereal time expression in degrees
	// see http://www.cdeagle.com/ccnum/pdf/demogast.pdf equation 1
	double GAST_degree = THETAm + dPSI*ossim::cosd(EPSILONm + dEPSILON);
	GAST_degree = GAST_degree - (int)(GAST_degree/360)*360;
	double GAST_arc = GAST_degree * 3.14159265358979 / 180.0;
	return GAST_arc;
}

//void ossimHjModel::imagingRay(const ossimDpt& image_point,
//							  ossimEcefRay&   image_ray) const
//{
//	bool runtime_dbflag = 0;
//	NEWMAT::Matrix satToOrbit;
//	ossimDpt iPt = image_point;
//	//iPt.samp += theSpotSubImageOffset.samp;
//	//iPt.line += theSpotSubImageOffset.line;
//
//	//
//	// 1. Establish time of line imaging:
//	//
//	//double t_line = theRefImagingTime +
//	//                theLineSamplingPeriod*(iPt.line - theRefImagingTimeLine);
//	double t_line = iPt.line;
//	double t = theRefImagingTime + (t_line - theRefImagingTimeLine) * 4.369e-3;
//	if (traceDebug() || runtime_dbflag)
//	{
//		ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::imagingRay():------------ BEGIN DEBUG PASS ---------------" << std::endl;
//		ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::imagingRay(): t_line = " << t_line << std::endl;
//	}
//	t_line += theLineOffset;
//
//	//
//	// 2. Interpolate ephemeris position and velocity (in ECF):
//	//
//	ossimEcefPoint  tempEcefPoint;
//	ossimEcefPoint  P_ecf;
//	theSupportData->getPositionEcf(t, P_ecf);
//	theSupportData->getVelocityEcf(t, tempEcefPoint);
//	//double omega = 7.292115e-5;
//	std::vector<double> vel_earth_rotation(3, 0.0);
//	//vel_earth_rotation[0] = omega *  P_ecf.y();
//	//vel_earth_rotation[1] = -omega * P_ecf.x();
//	ossimEcefVector V_ecf(tempEcefPoint.x() + vel_earth_rotation[0],
//		tempEcefPoint.y() + vel_earth_rotation[1],
//		tempEcefPoint.z() + vel_earth_rotation[2]);
//	if (traceDebug() || runtime_dbflag)
//	{
//		ossimNotify(ossimNotifyLevel_DEBUG)
//			<< "DEBUG:\n\tP_ecf = " << P_ecf
//			<< "\n\t V_ecf = " << V_ecf << std::endl;
//	}
//
//	//
//	// 3. Establish the look direction in Vehicle LSR space (S_sat).
//	//    ANGLES IN RADIANS
//	//
//	//ossim_float64 Psi_x;
//	//theSupportData->getPixelLookAngleX(iPt.samp, Psi_x);
//	//ossim_float64 Psi_y;
//	//theSupportData->getPixelLookAngleY(iPt.samp, Psi_y);
//	//if (traceDebug() || runtime_dbflag)
//	//{
//	//   ossimNotify(ossimNotifyLevel_DEBUG)
//	//      << "DEBUG:\n\t Psi_x = " << Psi_x
//	//      << "\n\t Psi_y = " << Psi_y << endl;
//	//}
//
//	//ossimColumnVector3d u_sat (-tan(Psi_y), tan(Psi_x), -(1.0 + theFocalLenOffset));
//	//if (traceDebug() || runtime_dbflag)
//	//{
//	//   ossimNotify(ossimNotifyLevel_DEBUG)
//	//      << "DEBUG \n\t u_sat = " << u_sat << endl;
//	//}
//
//	// focal plane 
//	// image-space view vector u_ccd in the telescope coordinate system
//	double midCCD = (theImageSize.samp+1) * 0.5 + theCcdOffset;
//	ossim_float64 x = (iPt.samp - midCCD) * theCcdStepLen;
//	ossim_float64 y = 0;
//	ossimColumnVector3d u_ccd(-x, y, -theFocalLen*(1.0 + theFocalLenOffset));
//
//	// 
//	NEWMAT::Matrix ccdToSat;
//	getCcdToSatRotation(ccdToSat);
//
//	//
//	// 4. Transform vehicle LSR space look direction vector to orbital LSR space
//	//    (S_orb):
//	//
//	computeSatToOrbRotation(satToOrbit, t_line);
//
//	ossimColumnVector3d u_orb = (ccdToSat*satToOrbit*u_ccd).unit();
//	if (traceDebug() || runtime_dbflag)
//	{
//		ossimNotify(ossimNotifyLevel_DEBUG)
//			<< "DEBUG:\n\t theSatToOrbRotation = " << satToOrbit
//			<< "\n\t u_orb = " << u_orb << endl;
//	}
//
//
//	//double omega = 7.292115e-5;
//	//double a = t * omega;
//	double a = calcGAST(t);
//	double sa = sin(a);
//	double ca = cos(a);
//
//	NEWMAT::Matrix j2000ToWGS84 = NEWMAT::Matrix(3, 3);
//	j2000ToWGS84 << ca << sa << 0.0
//	<< -sa  << ca << 0.0
//	<< 0.0 << 0.0 << 1;
//
//	ossimColumnVector3d u_ecf  = (j2000ToWGS84*u_orb).unit();
//	image_ray = ossimEcefRay(P_ecf, ossimEcefVector(u_ecf[0], u_ecf[1], u_ecf[2]));
//	return;
//	////
//	//// 5. Transform orbital LSR space look direction vector to ECF.
//	////
//	////   a. S_orb space Z-axis (Z_orb) is || to the ECF radial vector (P_ecf),
//	////   b. X_orb axis is computed as cross-product between velocity and radial,
//	////   c. Y_orb completes the orthogonal S_orb coordinate system.
//	////
//	//ossimColumnVector3d Z_orb (P_ecf.x(),
//	//	P_ecf.y(),
//	//	P_ecf.z());
//	//Z_orb = Z_orb.unit();
//
//	//ossimColumnVector3d X_orb = ossimColumnVector3d(V_ecf.x(),
//	//	V_ecf.y(),
//	//	V_ecf.z()).cross(Z_orb).unit();
//	//ossimColumnVector3d Y_orb = Z_orb.cross(X_orb);
//
//	//NEWMAT::Matrix orbToEcfRotation = NEWMAT::Matrix(3, 3);
//	//orbToEcfRotation << X_orb[0] << Y_orb[0] << Z_orb[0]
//	//<< X_orb[1] << Y_orb[1] << Z_orb[1]
//	//<< X_orb[2] << Y_orb[2] << Z_orb[2];
//
//
//	//ossimColumnVector3d u_ecf  = (orbToEcfRotation*u_orb);
//	//if (traceDebug() || runtime_dbflag)
//	//{
//	//	ossimNotify(ossimNotifyLevel_DEBUG)
//	//		<< "DEBUG:\n\t orbToEcfRotation = " << orbToEcfRotation
//	//		<< "\n\t u_ecf = " << u_ecf << endl;
//	//}
//
//	////
//	//// Establish the imaging ray given direction and origin:
//	////
//	//image_ray = ossimEcefRay(P_ecf, ossimEcefVector(u_ecf[0], u_ecf[1], u_ecf[2]));
//
//	//if (traceExec())
//	//{
//	//	ossimNotify(ossimNotifyLevel_DEBUG)
//	//		<< "DEBUG Spot5Model::imagingRay(): returning..." << std::endl;
//	//}
//}

void ossimHjModel::getExteriorParameters(const ossimDpt& image_point, 
										  NEWMAT::Matrix& rotationMat,
										  ossimEcefPoint& satPoint) const
{
	double t_line = image_point.line + theLineOffset;
	//
	// Interpolate ephemeris position and velocity (in ECF):
	//
	ossimEcefPoint  tempEcefPoint;
	theSupportData->getPositionEcf(t_line, satPoint);
	theSupportData->getVelocityEcf(t_line, tempEcefPoint);
	ossimEcefVector V_ecf(tempEcefPoint.x(),
		tempEcefPoint.y(),
		tempEcefPoint.z());

	// camera to body
	double ccd_roll, ccd_pitch, ccd_yaw;
	if (theSensorID.upcase().contains("HJ-1A CCD-1"))
	{
		ccd_roll = 15.182276 * M_PI / 180.0;
		ccd_pitch = 0.304668 * M_PI / 180.0;
		ccd_yaw = 0.106528 * M_PI / 180.0;
	}
	else if(theSensorID.upcase().contains("HJ-1A CCD-2"))
	{
		ccd_roll = -14.816663 * M_PI / 180.0;
		ccd_pitch = 0.552111 * M_PI / 180.0;
		ccd_yaw = 0.07 * M_PI / 180.0;
	}
	else if(theSensorID.upcase().contains("HJ-1B CCD-1"))
	{
		ccd_roll = 15.145556 * M_PI / 180.0;
		ccd_pitch = 0.441230 * M_PI / 180.0;
		ccd_yaw = 0.120700 * M_PI / 180.0;
	}
	else if(theSensorID.upcase().contains("HJ-1B CCD-2"))
	{
		ccd_roll = -15.015556 * M_PI / 180.0;
		ccd_pitch = 0.362064 * M_PI / 180.0;
		ccd_yaw = 0.300700 * M_PI / 180.0;
	}
	ccd_pitch = (ccd_pitch + theCcdPitchOffset);
	ccd_roll = (ccd_roll + theCcdRollOffset);
	ccd_yaw = (ccd_yaw + theCcdYawOffset);

	// body to orbit
	ossimDpt3d att(0.0, 0.0, 0.0);
	theSupportData->getAttitude(t_line, att);
	att.x += ccd_pitch;
	att.y += ccd_roll;
	att.z += ccd_yaw;
	double cp = cos(att.x);
	double sp = sin(att.x);
	double cr = cos(att.y);
	double sr = sin(att.y);
	double cy = cos(att.z);
	double sy = sin(att.z);
	NEWMAT::Matrix cameraToOrbit = NEWMAT::Matrix(3,3);
	cameraToOrbit << (cr*cy-sp*sr*sy) << (cr*sy+sp*sr*sy) << (-cp*sr)
		<< (-cp*sy) << (cp*cy) << (sp)
		<< (sr*cy+sp*cr*sy) << (sr*sy-sp*cr*cy) <<  cp*cr;

	// orbit to ECEF
	ossimColumnVector3d Z_orb (satPoint.x(),
		satPoint.y(),
		satPoint.z());
	Z_orb = -Z_orb.unit();
	ossimColumnVector3d X_orb = Z_orb.cross(ossimColumnVector3d(V_ecf.x(),
		V_ecf.y(),
		V_ecf.z())).unit();
	ossimColumnVector3d Y_orb = X_orb.cross(Z_orb);

	NEWMAT::Matrix orbToEcfRotation = NEWMAT::Matrix(3, 3);
	orbToEcfRotation << X_orb[0] << Y_orb[0] << Z_orb[0]
	<< X_orb[1] << Y_orb[1] << Z_orb[1]
	<< X_orb[2] << Y_orb[2] << Z_orb[2];

	rotationMat = orbToEcfRotation.t() * cameraToOrbit.t();
}

ossimEcefPoint ossimHjModel::collinearEquation_inverse(const ossimDpt& image_point,
												  const ossim_float64& ecef_Z,
												  const NEWMAT::Matrix& rotationMat,
												  const ossimEcefPoint& satPoint) const
{
	double samp = image_point.samp;
	double line = image_point.line + theLineOffset;

	// image to camera
	double x = computePhiY(samp)* (theFocalLen + theFocalOffset);
	double y = computePhiY(line)* (theFocalLen + theFocalOffset);

	double a1 = rotationMat.element(1, 1);
	double a2 = rotationMat.element(2, 1);
	double a3 = rotationMat.element(3, 1);
	double b1 = rotationMat.element(1, 2);
	double b2 = rotationMat.element(2, 2);
	double b3 = rotationMat.element(3, 2);
	double c1 = rotationMat.element(1, 3);
	double c2 = rotationMat.element(2, 3);
	double c3 = rotationMat.element(3, 3);


	double P1 = a1*x - a3*(theFocalLen + theFocalOffset);
	double P2 = b1*x - b3*(theFocalLen + theFocalOffset);
	double P3 = c1*x - c3*(theFocalLen + theFocalOffset);

	double ecef_X = satPoint.x() + (ecef_Z - satPoint.z()) * P1 / P3;
	double ecef_Y = satPoint.y() + (ecef_Z - satPoint.z()) * P2 / P3;

	return ossimEcefPoint(ecef_X, ecef_Y, ecef_Z);
}

ossimDpt ossimHjModel::collinearEquation_forward(const ossimEcefPoint& ecefPoint,
											  const NEWMAT::Matrix& rotationMat,
											  const ossimEcefPoint& satPoint) const
{
	double a1 = rotationMat.element(1, 1);
	double a2 = rotationMat.element(2, 1);
	double a3 = rotationMat.element(3, 1);
	double b1 = rotationMat.element(1, 2);
	double b2 = rotationMat.element(2, 2);
	double b3 = rotationMat.element(3, 2);
	double c1 = rotationMat.element(1, 3);
	double c2 = rotationMat.element(2, 3);
	double c3 = rotationMat.element(3, 3);

	double P1 = a1*(ecefPoint.x()- satPoint.x()) + b1*(ecefPoint.y() - satPoint.y()) + c1*(ecefPoint.z() - satPoint.z());
	double P2 = a2*(ecefPoint.x()- satPoint.x()) + b2*(ecefPoint.y() - satPoint.y()) + c2*(ecefPoint.z() - satPoint.z());
	double P3 = a3*(ecefPoint.x()- satPoint.x()) + b3*(ecefPoint.y() - satPoint.y()) + c3*(ecefPoint.z() - satPoint.z());

	double x = (theFocalLen + theFocalOffset) * P1 / P3;
	double y = (theFocalLen + theFocalOffset) * P2 / P3;

	return ossimDpt(x,y);
}

void ossimHjModel::collinearEquation_deriv(const ossimEcefPoint& ecefPoint,
											 const NEWMAT::Matrix& rotationMat,
											 const ossimEcefPoint& satPoint,
											 ossimDpt& dX, 
											 ossimDpt& dY,
											 ossimDpt& dZ) const
{
	double a1 = rotationMat.element(1, 1);
	double a2 = rotationMat.element(2, 1);
	double a3 = rotationMat.element(3, 1);
	double b1 = rotationMat.element(1, 2);
	double b2 = rotationMat.element(2, 2);
	double b3 = rotationMat.element(3, 2);
	double c1 = rotationMat.element(1, 3);
	double c2 = rotationMat.element(2, 3);
	double c3 = rotationMat.element(3, 3);

	double P1 = a1*(ecefPoint.x()- satPoint.x()) + b1*(ecefPoint.y() - satPoint.y()) + c1*(ecefPoint.z() - satPoint.z());
	double P2 = a2*(ecefPoint.x()- satPoint.x()) + b2*(ecefPoint.y() - satPoint.y()) + c2*(ecefPoint.z() - satPoint.z());
	double P3 = a3*(ecefPoint.x()- satPoint.x()) + b3*(ecefPoint.y() - satPoint.y()) + c3*(ecefPoint.z() - satPoint.z());
	double P32 = P3*P3;

	double dX_dx = (theFocalLen + theFocalOffset) * (a1 / P3 - P1 * a3 / P32);
	double dX_dy = (theFocalLen + theFocalOffset) * (a2 / P3 - P2 * a3 / P32);

	double dY_dx = (theFocalLen + theFocalOffset) * (b1 / P3 - P1 * b3 / P32);
	double dY_dy = (theFocalLen + theFocalOffset) * (b2 / P3 - P2 * b3 / P32);

	double dZ_dx = (theFocalLen + theFocalOffset) * (c1 / P3 - P1 * c3 / P32);
	double dZ_dy = (theFocalLen + theFocalOffset) * (c2 / P3 - P2 * c3 / P32);
	
	dX = ossimDpt(dX_dx, dX_dy);
	dY = ossimDpt(dY_dx, dY_dy);
	dZ = ossimDpt(dZ_dx, dZ_dy);
}

void ossimHjModel::lineSampleToWorld(const ossimDpt& image_point,
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

	lineSampleToWorld_collinearEquation(image_point, gpt);
}

void ossimHjModel::lineSampleToWorld_collinearEquation(const ossimDpt& image_point,
									  ossimGpt& worldPoint) const
{
	if(theSeedFunction.valid())
	{
		theSeedFunction->lineSampleToWorld(image_point, worldPoint);
		//theSeedFunction->worldToLineSample(worldPoint, image_point);
	}
	else
	{
		return;
	}
	ossimEcefPoint ecefPoint(worldPoint);

	double eps = 0.1;
	double err = DBL_MAX;
	double samp = image_point.samp;
	double line = image_point.line + theLineOffset;
	static const int    MAX_NUM_ITERATIONS = 20;
	int iters = 0;
	// image to camera
	double x = computePhiY(samp)* (theFocalLen + theFocalOffset);
	double y = computePhiY(line)* (theFocalLen + theFocalOffset);
	double ecef_Z = ecefPoint.z();
	do 
	{
		// ecef to camera
		NEWMAT::Matrix totalRotation;
		ossimEcefPoint satPoint;
		getExteriorParameters(image_point, totalRotation, satPoint);
		ecefPoint = collinearEquation_inverse(image_point, ecef_Z, totalRotation, satPoint);

		worldPoint = ossimGpt(ecefPoint, m_proj->getDatum());
		worldPoint.hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(worldPoint);
		ecefPoint = ossimEcefPoint(worldPoint);

		err = fabs(ecef_Z - ecefPoint.z());
		ecef_Z = ecefPoint.z();
		iters++;
	} while (err > eps && iters < MAX_NUM_ITERATIONS);

	if (iters >= MAX_NUM_ITERATIONS)
	{
		cout<<"warning: maximum iteration time reached!"<<endl;
	}
}

void ossimHjModel::worldToLineSample(const ossimGpt& worldPoint,
										 ossimDpt&       ip) const
{
	if(theSeedFunction.valid())
	{
		theSeedFunction->worldToLineSample(worldPoint, ip);
	}
	else
	{
		return;
	}
	ossimEcefPoint ecefPoint(worldPoint);

	double eps = 1e-3;
	double err = DBL_MAX;
	double samp = ip.samp;
	double line = ip.line + theLineOffset;

	do 
	{
		// image to camera
		double x = computePhiY(samp)* (theFocalLen + theFocalOffset);
		double y = computePhiY(line)* (theFocalLen + theFocalOffset);

		// ecef to camera
		NEWMAT::Matrix totalRotation;
		ossimEcefPoint satPoint;
		getExteriorParameters(ossimDpt(samp, line), totalRotation, satPoint);
		ossimDpt image_point_new = collinearEquation_forward(ecefPoint, totalRotation, satPoint);

		double delta_x = x - image_point_new.x;
		double delta_y = y - image_point_new.y;
		err = delta_x * delta_x + delta_y * delta_y;

		double midCCD = theImageSize.samp * 0.5;
		double k = (theCcdStepLen)/(theFocalLen+theFocalOffset) * 1.0e0;
		double kx = (samp - midCCD) / midCCD;
		double ay0 = 0.0;
		double ay1 = 0.0;
		double ay2 = 0.0;
		double ay3 = 0.0;
		double ay4 = 0.0;
		double ay5 = 0.0;
		double ay6 = 0.0;
		if (theSensorID.upcase().contains("HJ-1A CCD-1"))
		{
		}
		else if(theSensorID.upcase().contains("HJ-1A CCD-2"))
		{
		}
		else if(theSensorID.upcase().contains("HJ-1B CCD-1"))
		{
		}
		else if(theSensorID.upcase().contains("HJ-1B CCD-2"))
		{
		}

		ay0 += theAY0Offset;
		ay1 += theAY1Offset;
		ay2 += theAY2Offset;
		ay3 += theAY3Offset;
		ay4 += theAY4Offset;
		ay5 += theAY5Offset;
		ay6 += theAY6Offset;

		ossim_float64 dx_dsamp = 1.0
			+ ay1
			+ ay2 * kx
			+ ay3 * kx * kx
			+ ay4 * kx * kx * kx
			+ ay5 * kx * kx * kx * kx
			+ ay6 * kx * kx * kx * kx * kx;
		dx_dsamp = dx_dsamp * midCCD * theCcdStepLen;

		samp += delta_x / dx_dsamp;
		line += delta_y;
	} while (err > eps);

	ip = ossimDpt(samp, line - theLineOffset);
}

void ossimHjModel::lineSampleHeightToWorld(const ossimDpt& image_point,
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
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::lineSampleHeightToWorld(): returning..." << std::endl;
      return;
   }
   //***
   // First establish imaging ray from image point:
   //***
   ossimEcefRay imaging_ray;
   imagingRay(image_point, imaging_ray);
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

// ossimDpt ossimHjModel::extrapolate (const ossimGpt& gp) const
// {
//     ossimDpt temp;

//     temp.makeNan();

//     return temp;

//   ossimDpt tempGpt = gp;
//   ossimDpt dest;
//   theGroundToImageMap.map(tempGpt, dest);

//  return dest;

// }

// ossimGpt ossimHjModel::extrapolate (const ossimDpt& ip,
// 				       const double& height) const
// {
//   return ossimGpt(ossim::nan(), ossim::nan(), ossim::nan(), 0);

//    ossimDpt dest;

//    theImageToGroundMap.map(ip, dest);


//    return ossimGpt(dest.lat, dest.lon, ossim::nan(), origin().datum());
// }

bool
ossimHjModel::setupOptimizer(const ossimString& init_file)
{
   ossimFilename qvProcFile = init_file;
   ossimFilename geomFile = init_file;
   geomFile = geomFile.setExtension("geom");
   bool tryKwl = false;

   if(!qvProcFile.exists())
   {
      qvProcFile = geomFile.path();
      qvProcFile = qvProcFile.dirCat(ossimFilename("DESC.XML"));
      if(qvProcFile.exists() == false)
      {
         qvProcFile = geomFile.path();
         qvProcFile = qvProcFile.dirCat(ossimFilename("desc.xml"));
      }
   }
   if(qvProcFile.exists())
   {
      ossimRefPtr<ossimQVProcSupportData> meta = new ossimQVProcSupportData;
      if(meta->loadXmlFile(qvProcFile))
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
ossimHjModel::initFromMetadata(ossimQVProcSupportData* sd)
{
   // init parms
   theSupportData        = sd;
   theMetaDataFile       = "NOT ASSIGNED";
   theIllumAzimuth       = 0.0;
   theIllumElevation     = 0.0;
   thePositionError      = 0.0;
   theLineSamplingPeriod = 0.0;
   theRefImagingTime      = 0.0;
   theRefImagingTimeLine  = 0.0;
//   theSatToOrbRotation   = 0.0; //matrix
//   theOrbToEcfRotation   = 0.0; //matrix
   //theRollOffset         = 0.0;
   //thePitchOffset        = 0.0;
   //theYawOffset          = 0.0;
   theCcdRollOffset           = 0.0;
   theCcdPitchOffset          = 0.0;
   theCcdYawOffset            = 0.0;
   theXOffset                 = 0.0;
   theYOffset                 = 0.0;
   theZOffset                 = 0.0;
   theFocalOffset        = 0.0;
   theAY0Offset     = 0.0;
   theAY1Offset     = 0.0;
   theAY2Offset     = 0.0;
   theAY3Offset     = 0.0;
   theAY4Offset     = 0.0;
   theAY5Offset     = 0.0;
   theAY6Offset     = 0.0;
   theLineOffset     = 0.0;


   //---
   // Instantiate the support data classes after establishing the filenames:
   //---
   loadSupportData();
   if (getErrorStatus() != ossimErrorCodes::OSSIM_OK)
   {
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimHjModel::initFromMetadata(dimap_file): returning with error..." << std::endl;
      return false;
   }

   //---
   // initialize remaining data members:
   //---
   initAdjustableParameters();
   updateModel();
   return true;
}


void ossimHjModel::setImageRect(const ossimDrect& roi)
{
	theImageClipRect = roi;
	theRefImgPt = theImageClipRect.midPoint();

	ossimGpt p1;
	ossimGpt p2;
	ossimGpt p3;
	ossimGpt p4;

	// I need to find the nominal scale of the spot 5 dataset

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
}

//ossimDpt
//	ossimHjModel::getForwardDeriv(int iAtt, int iComponent, const ossimGpt& gpos, double hdelta)
//{   
//	double den = 0.5/hdelta;
//	ossimDpt res;
//	if (0 == iComponent)
//	{
//		double middle = theSupportData->theAttitudeBias[iAtt].x;
//		theSupportData->theAttitudeBias[iAtt].x = middle + hdelta;
//		//res = inverse(gpos);
//		res = forward(gpos);
//		theSupportData->theAttitudeBias[iAtt].x = middle - hdelta;
//		//res -= inverse(gpos);
//		res -= forward(gpos);
//		res = res*den;
//		theSupportData->theAttitudeBias[iAtt].x = middle;
//	}
//	else if(1 == iComponent)
//	{
//		double middle = theSupportData->theAttitudeBias[iAtt].y;
//		theSupportData->theAttitudeBias[iAtt].y = middle + hdelta;
//		//res = inverse(gpos);
//		res = forward(gpos);
//		theSupportData->theAttitudeBias[iAtt].y = middle - hdelta;
//		//res -= inverse(gpos);
//		res -= forward(gpos);
//		res = res*den;
//		theSupportData->theAttitudeBias[iAtt].y = middle;
//	}
//	else
//	{
//		double middle = theSupportData->theAttitudeBias[iAtt].z;
//		theSupportData->theAttitudeBias[iAtt].z = middle + hdelta;
//		//res = inverse(gpos);
//		res = forward(gpos);
//		theSupportData->theAttitudeBias[iAtt].z = middle - hdelta;
//		//res -= inverse(gpos);
//		res -= forward(gpos);
//		res = res*den;
//		theSupportData->theAttitudeBias[iAtt].z = middle;
//	}
//	return res;
//}
//
//void
//	ossimHjModel::buildNormalEquation(const ossimTieGptSet& tieSet,
//	NEWMAT::SymmetricMatrix& A,
//	NEWMAT::ColumnVector& residue,
//	NEWMAT::ColumnVector& projResidue,
//	double pstep_scale)
//{
//	//goal:       build Least Squares system
//	//constraint: never store full Jacobian matrix in memory (can be huge)
//	//            so we build the matrices incrementally
//	// the system can be built using forward() or inverse() depending on the projection capabilities : useForward()
//	//
//	//TBD : add covariance matrix for each tie point	
//	//init
//	int np = theSupportData->getNumAttSamples()*3;
//	int dimObs;
//	bool useImageObs = useForward(); //caching
//	if (useImageObs)
//	{
//		dimObs = 2; //image observation
//	} else {
//		dimObs = 3; //ground observations
//	}
//	int no = dimObs * tieSet.size(); //number of observations
//
//	A.ReSize(np);
//	residue.ReSize(no);
//	projResidue.ReSize(np);
//	//Zeroify matrices that will be accumulated
//	A           = 0.0;
//	projResidue = 0.0;
//
//	const vector<ossimRefPtr<ossimTieGpt> >& theTPV = tieSet.getTiePoints();
//	vector<ossimRefPtr<ossimTieGpt> >::const_iterator tit;
//	unsigned long c=1;
//
//	//image observations 
//	std::vector<ossimDpt> imDerp(np);
//	ossimDpt resIm;
//	// loop on tie points
//	for (tit = theTPV.begin() ; tit != theTPV.end() ; ++tit)
//	{
//		//compute residue
//		resIm = (*tit)->tie - forward(*(*tit));
//		residue(c++) = resIm.x;
//		residue(c++) = resIm.y;
//
//		//compute all image derivatives regarding parametres for the tie point position
//		for(int p=0;p<np;p+=3)
//		{
//			int iAtt = p / 3;
//			imDerp[p] = getForwardDeriv( iAtt , 0, *(*tit) , pstep_scale);
//			imDerp[p+1] = getForwardDeriv( iAtt , 1, *(*tit) , pstep_scale);
//			imDerp[p+2] = getForwardDeriv( iAtt , 2, *(*tit) , pstep_scale);
//		}
//
//		//compute influence of tie point on all sytem elements
//		for(int p1=0;p1<np;++p1)
//		{        
//			//proj residue: J * residue
//			projResidue.element(p1) += imDerp[p1].x * resIm.x + imDerp[p1].y * resIm.y;
//
//			//normal matrix A = transpose(J)*J
//			for(int p2=p1;p2<np;++p2)
//			{
//				A.element(p1,p2) += imDerp[p1].x * imDerp[p2].x + imDerp[p1].y * imDerp[p2].y;
//			}
//		}
//	}
//}
//
//double
//	ossimHjModel::optimizeFit(const ossimTieGptSet& tieSet, double* /* targetVariance */)
//{
//	//use a simple Levenberg-Marquardt non-linear optimization
//	//note : please limit the number of tie points
//	//
//	//INPUTS: requires Jacobian matrix (partial derivatives with regards to parameters)
//	//OUPUTS: will also compute parameter covariance matrix
//	//
//	//TBD: use targetVariance!
//
//	int np = theSupportData->getNumAttSamples()*3;
//	int nobs = tieSet.size();
//
//	//setup initail values
//	int iter=0;
//	int iter_max = 200;
//	double minResidue = 1e-10; //TBC
//	double minDelta = 1e-10; //TBC
//
//	//build Least Squares initial normal equation
//	// don't waste memory, add samples one at a time
//	NEWMAT::SymmetricMatrix A;
//	NEWMAT::ColumnVector residue;
//	NEWMAT::ColumnVector projResidue;
//	double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1
//	buildNormalEquation(tieSet, A, residue, projResidue, deltap_scale);
//	double ki2=residue.SumSquare();
//
//	//get current adjustment (between -1 and 1 normally) and convert to ColumnVector
//	NEWMAT::ColumnVector cparm(np), nparm(np);
//	for(int n=0;n<np;n+=3)
//	{
//		int iAtt = n / 3;
//		cparm(n+1) = theSupportData->theAttitudeBias[iAtt].x;
//		cparm(n+2) = theSupportData->theAttitudeBias[iAtt].y;
//		cparm(n+3) = theSupportData->theAttitudeBias[iAtt].z;
//	}
//
//	double damping_speed = 2.0;
//	//find max diag element for A
//	double maxdiag=0.0;
//	for(int d=1;d<=np;++d) {
//		if (maxdiag < A(d,d)) maxdiag=A(d,d);
//	}
//	double damping = 1e-3 * maxdiag;
//	double olddamping = 0.0;
//	bool found = false;
//
//	//DEBUG TBR
//	// cout<<"rms="<<sqrt(ki2/nobs)<<" ";
//	// cout.flush();
//
//	while ( (!found) && (iter < iter_max) ) //non linear optimization loop
//	{
//		bool decrease = false;
//
//		do
//		{
//			//add damping update to normal matrix
//			for(int d=1;d<=np;++d) A(d,d) += damping - olddamping;
//			olddamping = damping;
//
//			NEWMAT::ColumnVector deltap = solveLeastSquares(A, projResidue);
//
//			if (deltap.NormFrobenius() <= minDelta) 
//			{
//				found = true;
//			} else {
//				//update adjustment
//				nparm = cparm + deltap;
//				for(int n=0;n<np;++n)
//				{
//					setAdjustableParameter(n, nparm(n+1), false); //do not update now, wait
//				}
//				updateModel();
//
//				//check residue is reduced
//				NEWMAT::ColumnVector newresidue = getResidue(tieSet);
//				double newki2=newresidue.SumSquare();
//				double res_reduction = (ki2 - newki2) / (deltap.t()*(deltap*damping + projResidue)).AsScalar();
//				//DEBUG TBR
//				cout<<sqrt(newki2/nobs)<<" ";
//				cout.flush();
//
//				if (res_reduction > 0)
//				{
//					//accept new parms
//					cparm = nparm;
//					ki2=newki2;
//
//					deltap_scale = max(1e-15, deltap.NormInfinity()*1e-4);
//
//					buildNormalEquation(tieSet, A, residue, projResidue, deltap_scale);
//					olddamping = 0.0;
//
//					found = ( projResidue.NormInfinity() <= minResidue );
//					//update damping factor
//					damping *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
//					damping_speed = 2.0;
//					decrease = true;
//				} else {
//					//cancel parameter update
//					for(int n=0;n<np;n+=3)
//					{
//						int iAtt = n / 3;
//						theSupportData->theAttitudeBias[iAtt].x = nparm(n+1);
//						theSupportData->theAttitudeBias[iAtt].y = nparm(n+2);
//						theSupportData->theAttitudeBias[iAtt].z = nparm(n+3);
//					}
//					updateModel();
//
//					damping *= damping_speed;
//					damping_speed *= 2.0;
//				}
//			}
//		} while (!decrease && !found);
//		++iter;
//	}
//
//	//DEBUG TBR
//	cout<<endl;
//
//	//compute parameter correlation
//	// use normal matrix inverse
//	//TBD
//
//	return ki2/nobs;
//}