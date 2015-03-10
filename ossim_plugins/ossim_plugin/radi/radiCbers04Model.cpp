//*******************************************************************
//
// License:  See top level LICENSE.txt file.
//
// Author:  Oscar Kramer (ossim port by D. Burken)
//
// Description:
//
// Contains definition of class radiCbers04Model.
//
//*****************************************************************************
// $Id: radiCbers04Model.cpp 19658 2011-05-26 13:16:06Z gpotts $

#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

#include <radi/ossimQVProcSupportData.h>
#include <radi/radiCbers04Model.h>
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
#include <ossim/elevation/ossimElevManager.h>

#include <ossim/projection/ossimRpcSolver.h>
#include <ossim/imaging/ossimImageGeometry.h>

//---
// Define Trace flags for use within this file:
//---
#include <ossim/base/ossimTrace.h>

namespace ossimplugins
{
	static ossimTrace traceExec  ("radiCbers04Model:exec");
	static ossimTrace traceDebug ("radiCbers04Model:debug");
	RTTI_DEF1(radiCbers04Model, "radiCbers04Model", ossimSensorModel);

static const ossim_int32 MODEL_VERSION_NUMBER = 1;

static const char* PARAM_NAMES[] = { "ccd_roll_offset",
                                     "ccd_pitch_offset",
                                     "ccd_yaw_offset",
									 "x_offset",
									 "y_offset",
									 "z_offset",
									 "focal_offset",
									 "ccd_offset",
									 "ax0_offset",
									 "ax1_offset",
									 "ax2_offset",
									 "ax3_offset",
									 "ay0_offset",
									 "ay1_offset",
									 "ay2_offset",
									 "ay3_offset",
									 "ay4_offset",
									 "ay5_offset",
									 "ay6_offset",
									 "az0_offset",
									 "az1_offset",
									 "az2_offset",
									 "az3_offset",
									 "line_offset",
									 "ccd_rot_center_offset",
									 "ccd_rot_angle",
									 "ccd_center_x_offset",
									 "ccd_center_y_offset",
									 "ccd_k1",
									 "ccd_k2",
									 "ccd_p1",
									 "ccd_p2"};

static const char* PARAM_UNITS[] = { 
									 "degrees",   // degrees
									 "degrees",   // degrees
									 "degrees",   // degrees,
									 "none",
									 "none",
									 "none",
									 "meter",
									 "pixel",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "pixel",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none",
									 "none"}; 

static const ossim_float64 SIGMA[] = { 1.0e-4,  // ccd_roll_offset delta degrees
									   1.0e-4,  // ccd_pitch_offset delta degrees
									   1.0e-4,  // ccd_yaw_offset delta degrees
									   1.0e-4,  // x_offset
									   1.0e-4,  // y_offset
									   1.0e-4,  // z_offset
									   1.0e-4,	 // focal_offset
									   1.0,	// ccd_offset
									   1.0e-4,	// ax0_offset
									   1.0e-4,	// ax1_offset
									   1.0e-4,	// ax2_offset
									   1.0e-4,	// ax3_offset
									   1.0e-4,	// ay0_offset
									   1.0e-4,	// ay1_offset
									   1.0e-4,	// ay2_offset
									   1.0e-4,	// ay3_offset
									   1.0e-4,	// ay4_offset
									   1.0e-4,	// ay5_offset
									   1.0e-4,	// ay6_offset
									   1.0e-4,	// az0_offset
									   1.0e-4,	// az1_offset
									   1.0e-4,	// az2_offset
									   1.0e-4,	// az3_offset
									   1.0e0,	// line_offset
									   1.0e0,	// ccd_rot_center_offset
									   1.0e-4,	// ccd_rot_angle
									   1.0e0,	// ccd_center_x_offset
									   1.0e0,	// ccd_center_y_offset
									   1.0e-2,	// ccd_k1
									   1.0e-2,	// ccd_k2
									   1.0e-2,	// ccd_p1
									   1.0e-2	// ccd_p2
};

radiCbers04Model::radiCbers04Model()
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
   theCcdOffset     (0.0),
   theAX0Offset	         (0.0),
   theAX1Offset	         (0.0),
   theAX2Offset	         (0.0),
   theAX3Offset	         (0.0),
   theAY0Offset	         (0.0),
   theAY1Offset	         (0.0),
   theAY2Offset	         (0.0),
   theAY3Offset	         (0.0),
   theAY4Offset	         (0.0),
   theAY5Offset	         (0.0),
   theAY6Offset	         (0.0),
   theAZ0Offset	         (0.0),
   theAZ1Offset	         (0.0),
   theAZ2Offset	         (0.0),
   theAZ3Offset	         (0.0),
   theLineOffset         (0.0),
   theRollOffset         (0.0),
   thePitchOffset         (0.0),
   theYawOffset         (0.0),
   theFocalLen(1.4078),	// meters
   theCcdRotCenterOffset(0.0),
   theCcdRotAngle(0.0),
   theCcdCenterXOffset(0.0),
   theCcdCenterYOffset(0.0),
   theCcdK1(0.0),
   theCcdK2(0.0),
   theCcdP1(0.0),
   theCcdP2(0.0),
   //theFocalLen(1.3831539),
   theCcdStepLen(0.0),	// meters
   theApproximateModel(NULL)
{
   initAdjustableParameters();
}

radiCbers04Model::radiCbers04Model(ossimQVProcSupportData* sd)
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
   theCcdOffset     (0.0),
   theAX0Offset	         (0.0),
   theAX1Offset	         (0.0),
   theAX2Offset	         (0.0),
   theAX3Offset	         (0.0),
   theAY0Offset	         (0.0),
   theAY1Offset	         (0.0),
   theAY2Offset	         (0.0),
   theAY3Offset	         (0.0),
   theAY4Offset	         (0.0),
   theAY5Offset	         (0.0),
   theAY6Offset	         (0.0),
   theAZ0Offset	         (0.0),
   theAZ1Offset	         (0.0),
   theAZ2Offset	         (0.0),
   theAZ3Offset	         (0.0),
   theLineOffset         (0.0),
   theRollOffset         (0.0),
   thePitchOffset         (0.0),
   theYawOffset         (0.0),
   theFocalLen(1.4078),	// meters
   theCcdRotCenterOffset(0.0),
   theCcdRotAngle(0.0),
   theCcdCenterXOffset(0.0),
   theCcdCenterYOffset(0.0),
   theCcdK1(0.0),
   theCcdK2(0.0),
   theCcdP1(0.0),
   theCcdP2(0.0),
   //theFocalLen(1.3831539),
   theCcdStepLen(0.0),	// meters
   theApproximateModel(NULL)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model(desc_file) Constructor: entering..." << std::endl;

   //---
   // Instantiate the support data classes after establishing the filenames:
   //---
   loadSupportData();
   if (getErrorStatus() != ossimErrorCodes::OSSIM_OK)
   {
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model(desc_file) Constructor: returning with error..." << std::endl;
      return;
   }

   //---
   // initialize remaining data members:
   //---
   initAdjustableParameters();
   updateModel();

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model(desc_file) Constructor: returning..." << std::endl;
}

radiCbers04Model::radiCbers04Model(radiCbers04SupportData* sd)
	:
	ossimSensorModel      (),
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
	theCcdOffset     (0.0),
	theAX0Offset	         (0.0),
	theAX1Offset	         (0.0),
	theAX2Offset	         (0.0),
	theAX3Offset	         (0.0),
	theAY0Offset	         (0.0),
	theAY1Offset	         (0.0),
	theAY2Offset	         (0.0),
	theAY3Offset	         (0.0),
	theAY4Offset	         (0.0),
	theAY5Offset	         (0.0),
	theAY6Offset	         (0.0),
	theAZ0Offset	         (0.0),
	theAZ1Offset	         (0.0),
	theAZ2Offset	         (0.0),
	theAZ3Offset	         (0.0),
	theLineOffset         (0.0),
	theRollOffset         (0.0),
	thePitchOffset         (0.0),
	theYawOffset         (0.0),
	theFocalLen(1.700),	// meters
	theCcdRotCenterOffset(0.0),
	theCcdRotAngle(0.0),
	theCcdCenterXOffset(0.0),
	theCcdCenterYOffset(0.0),
	theCcdK1(0.0),
	theCcdK2(0.0),
	theCcdP1(0.0),
	theCcdP2(0.0),
	//theFocalLen(1.3831539),
	theCcdStepLen(0.0),	// meters
	theApproximateModel(NULL)
{
	if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model(desc_file) Constructor: entering..." << std::endl;
	ossimKeywordlist kwl;
	sd->saveState(kwl);
	theSupportData = new ossimQVProcSupportData();
	theSupportData->loadState(kwl);
	//---
	// Instantiate the support data classes after establishing the filenames:
	//---
	loadSupportData();
	if (getErrorStatus() != ossimErrorCodes::OSSIM_OK)
	{
		if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model(desc_file) Constructor: returning with error..." << std::endl;
		return;
	}

	//---
	// initialize remaining data members:
	//---
	initAdjustableParameters();
	updateModel();

	if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model(desc_file) Constructor: returning..." << std::endl;
}

//*****************************************************************************
//  DESTRUCTOR: ~radiCbers04Model()
//
//*****************************************************************************
radiCbers04Model::~radiCbers04Model()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG DESTRUCTOR: ~radiCbers04Model(): entering..." << std::endl;

   theSupportData = 0;
   
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG DESTRUCTOR: ~radiCbers04Model(): returning..." << std::endl;
}

radiCbers04Model::radiCbers04Model(const radiCbers04Model& rhs)
   :ossimSensorModel(rhs)
{
   if(rhs.theSupportData.valid())
   {
      theSupportData = (ossimQVProcSupportData*)rhs.theSupportData->dup();
   }
   loadSupportData();
   updateModel();
}


void radiCbers04Model::getCcdToSatRotation(NEWMAT::Matrix& result)const
{
	result = NEWMAT::Matrix(3,3);
	double roll, pitch, yaw;
	roll = 0.0;
	pitch = 0.0;
	yaw = 0.0;
	if (theSensorID.upcase().contains("P5M"))
	{
		roll = 0.0;
		pitch = 0.0;
		yaw = 0.0;
	}
	else if(theSensorID.upcase().contains("P10"))
	{
		roll = 0.0;
		pitch = 0.0;
		yaw = 0.0;
	}
	else if(theSensorID.upcase().contains("MUX"))
	{
		//roll = -77.8201e-4;
		//pitch = -92.9642e-4;
		//yaw = 32.1378e-4;
	}
	else if(theSensorID.upcase().contains("WFI"))
	{
		roll = 0.0;
		pitch = 0.0;
		yaw = 0.0;
	}
	else if (theSensorID.upcase().contains("IRS"))
	{
		roll = 0.0;
		pitch = 0.0;
		yaw = 0.0;
	}
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

void radiCbers04Model::computeSatToOrbRotation(NEWMAT::Matrix& result, ossim_float64 t)const
{
   if (traceExec())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
      << "DEBUG radiCbers04Model::computeSatToOrbRotation(): entering..."
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

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::computeSatToOrbRotation(): returning..." << std::endl;
}

#if 0
//*****************************************************************************
//  METHOD
//*****************************************************************************
void radiCbers04Model::computeSatToOrbRotation(ossim_float64 t)const
{
   if (traceExec())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "DEBUG radiCbers04Model::computeSatToOrbRotation(): entering..."
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


    if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::computeSatToOrbRotation(): returning..." << std::endl;
}
#endif

ossim_float64 radiCbers04Model::computePhiX(double samp) const
{
	double ps = theCcdStepLen;
	double midCCD = theImageSize.samp * 0.5;
	double x = (samp - midCCD - theCcdOffset) * 1.0e-0;
	double kx = x / midCCD;
	double ax0 = 0.0;
	double ax1 = 0.0;
	double ax2 = 0.0;
	double ax3 = 0.0;

	if (theSensorID.upcase().contains("P5M"))
	{
		ax0 = 1.4121e-4;
	}
	else if (theSensorID.upcase().contains("P10"))
	{
		ax0 = 0.0;
	}
	else if (theSensorID.upcase().contains("MUX"))
	{
		ax0 = 7.9022e-4;
	}
	else if (theSensorID.upcase().contains("WFI"))
	{
		ax0 = 0.0;
	}
	else if (theSensorID.upcase().contains("IRS"))
	{
		ax0 = 0.0;
	}

	ax0 += theAX0Offset;
	ax1 += theAX1Offset;
	ax2 += theAX2Offset;
	ax3 += theAX3Offset;

	//ossim_float64 phiX = ax0 + ax1 * samp
	//	+ ax2 * samp * samp + ax3 * samp * samp *samp;

	//return phiX;

	ossim_float64 phiX = ax0
		+ ax1 * kx
		+ ax2 * kx * kx
		+ ax3 * kx * kx * kx
		+ theAZ2Offset * kx * kx * kx * kx
		+ theAZ3Offset * kx * kx * kx * kx * kx;
	double z = theFocalLen + theAZ0Offset 
		+ theAZ1Offset * kx;
		//+ theAZ2Offset * kx * kx
		//+ theAZ3Offset * kx * kx * kx;
	phiX = phiX * midCCD * ps / (theFocalLen + theFocalOffset);
	return phiX;
}

ossim_float64 radiCbers04Model::computePhiY(double samp) const
{
	double ps = theCcdStepLen;

	double midCCD = theImageSize.samp * 0.5;
	//double k = (theCcdStepLen)/(theFocalLen+theFocalOffset) * 1.0e0;
	double x = (samp - midCCD - theCcdOffset) * 1.0e-0;
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

	if (theSensorID.upcase().contains("P5M"))
	{
		ay0 = -2.35115e-4;
		ay1 = -16.8039e-4;
		ay2 = 2.0509e-4;
		ay3 = -36.2876e-4;
		ay4 = -4.24158e-4;
		ay5 = 0.0;
	}
	else if(theSensorID.upcase().contains("P10"))
	{
		ay0 = 0.0;
		ay1 = 0.0;
		ay2 = 0.0;
		ay3 = 0.0;
		ay4 = 0.0;
		ay5 = 0.0;
	}
	else if(theSensorID.upcase().contains("MUX"))
	{
		ay0 = -6.24166e-4;
		ay1 = 9.74598e-4;
		ay2 = 23.0051e-4;
		ay3 = -6.9902e-4;
		ay4 = -22.5787e-4;
		ay5 = 0.0;
	}
	else if(theSensorID.upcase().contains("WFI"))
	{
		ay0 = 0.0;
		ay1 = 0.0;
		ay2 = 0.0;
		ay3 = 0.0;
		ay4 = 0.0;
		ay5 = 0.0;
	}
	else if (theSensorID.upcase().contains("IRS"))
	{
		ay0 = 0.0;
		ay1 = 0.0;
		ay2 = 0.0;
		ay3 = 0.0;
		ay4 = 0.0;
		ay5 = 0.0;
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
	double z = theFocalLen + theAZ0Offset 
		+ theAZ1Offset * kx;
		//+ theAZ2Offset * kx * kx
		//+ theAZ3Offset * kx * kx * kx;
	//ossim_float64 phiY = ay0 * x
	//	+ ay1 * x * x * x
	//	+ ay2 * x * x * x * x * x
	//	+ ay3 * x * x * x * x * x * x * x;
	phiY = phiY * midCCD * ps / (theFocalLen + theFocalOffset);
	return phiY;
}

void radiCbers04Model::create3DGridPoints(const ossimDrect& imageBounds,
									   double height,
						ossimTieGptSet*& pTieGptSet,
						ossim_uint32 xSamples,
						ossim_uint32 ySamples,
						bool latlon,
						bool shiftTo0Flag)
{
	if (NULL == pTieGptSet)
	{
		pTieGptSet = new ossimTieGptSet;
	}
	ossim_uint32 x,y;
	ossim_float64 w = imageBounds.width();
	ossim_float64 h = imageBounds.height();
	ossimGpt gpt;
	ossimGpt defaultGround;
	if(ySamples < 1) ySamples = 12;
	if(xSamples < 1) xSamples = 12;
	srand(time(0));
	double xnorm;
	double ynorm;
	ossimDpt ul = imageBounds.ul();
	ossimDpt shiftTo0(-ul.x,
		-ul.y);
	for(y = 0; y < ySamples; ++y)
	{
		for(x = 0; x < xSamples; ++x)
		{
			ossimDpt imagePoint;
			if(ySamples > 1)
			{
				ynorm = (double)y/(double)(ySamples - 1);
			}
			else
			{
				ynorm = 0.0;
			}
			if(xSamples > 1)
			{
				xnorm = (double)x/(double)(xSamples - 1);
			}
			else
			{
				xnorm = 0.0;
			}

			ossimDpt dpt((w-1)*xnorm + ul.x,
				(h-1)*ynorm + ul.y);

			lineSampleHeightToWorld(dpt, height, gpt);
			if(shiftTo0Flag)
			{
				imagePoint = dpt + shiftTo0;
			}
			else
			{
				imagePoint = dpt;
			}
			gpt.hgt = height;

			ossimString strId;
			char tmpStr[256];
			sprintf(tmpStr, "%d", y * xSamples + x + 1);
			strId = tmpStr;
			ossimTieGpt *aTiePt = new ossimTieGpt(gpt, imagePoint, 1.0, strId);
			pTieGptSet->addTiePoint(aTiePt);
		}
	}
}

void radiCbers04Model::updateApproximateModel()
{
	// statistic the max and min Height
	ossim_uint32 xSamples = 20;
	ossim_uint32 ySamples = 20;

	double max_Height = -1.0e10;
	double min_Height = 1.0e10;
	ossim_uint32 x,y;
	ossim_float64 w = theImageClipRect.width();
	ossim_float64 h = theImageClipRect.height();
	ossimGpt gpt;
	ossimGpt defaultGround;
	if(ySamples < 1) ySamples = 12;
	if(xSamples < 1) xSamples = 12;

	double xnorm;
	double ynorm;
	ossimDpt ul = theImageClipRect.ul();
	ossimDpt shiftTo0(-ul.x, -ul.y);
	for(y = 0; y < ySamples; ++y)
	{
		for(x = 0; x < xSamples; ++x)
		{
			ossimDpt imagePoint;
			if(ySamples > 1)
			{
				ynorm = (double)y/(double)(ySamples - 1);
			}
			else
			{
				ynorm = 0.0;
			}
			if(xSamples > 1)
			{
				xnorm = (double)x/(double)(xSamples - 1);
			}
			else
			{
				xnorm = 0.0;
			}

			ossimDpt dpt((w-1)*xnorm + ul.x,
				(h-1)*ynorm + ul.y);

			lineSampleToWorld(dpt, gpt);
			if (!gpt.hasNans())
			{
				if (gpt.height() > max_Height)
				{
					max_Height = gpt.height();
				}
				if (gpt.height() < min_Height)
				{
					min_Height = gpt.height();
				}
			}
		}
	}

	min_Height -= 500.0;
	max_Height += 500.0;
	ossimTieGptSet *gptSet = NULL;
	int nLevels = 5;
	for (int i=0;i < nLevels;++i)
	{
		double hgt = min_Height + i*(max_Height-min_Height)/(nLevels-1);
		create3DGridPoints(theImageClipRect, hgt, gptSet, 15, 15, true, false);
	}

	ossimRpcSolver *solver = new ossimRpcSolver(true, false);
	int num = (int)gptSet->getTiePoints().size();
	vector < ossimDpt > imagePoints(num);
	vector < ossimGpt > groundControlPoints(num);
	for(int i = 0;i < num;i++)
	{
		groundControlPoints[i] = gptSet->getTiePoints()[i]->getGroundPoint();
		imagePoints[i] = gptSet->getTiePoints()[i]->getImagePoint();
	}
	solver->solveCoefficients(imagePoints, groundControlPoints);

	ossimImageGeometry *imageGeom = solver->createRpcModel();
	ossimKeywordlist geom;
	imageGeom->saveState(geom);
	theApproximateModel = new ossimRpcModel();
	theApproximateModel->loadState(geom, "projection.");
}

//*****************************************************************************
// PUBLIC METHOD: radiCbers04Model::updateModel()
//
//  Updates the model parameters given the normalized adjustable parameter
//  array.
//
//*****************************************************************************
void radiCbers04Model::updateModel()
{
   clearErrorStatus();

   try
   {
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::updateModel(): entering..." << std::endl;

      if(getNumberOfAdjustableParameters() < 1)
      {
         theCcdRollOffset       = 0;
         theCcdPitchOffset      = 0;
         theCcdYawOffset        = 0;
		 theXOffset   = 0;
		 theYOffset   = 0;
		 theZOffset   = 0;
		 theFocalOffset    = 0;
		 theCcdOffset = 0;
		 theAX0Offset = 0;
		 theAX1Offset	= 0;
		 theAX2Offset = 0;
		 theAX3Offset = 0;
		 theAY0Offset = 0;
		 theAY1Offset	= 0;
		 theAY2Offset = 0;
		 theAY3Offset = 0;
		 theAY4Offset = 0;
		 theAY5Offset = 0;
		 theAY6Offset = 0;
		 theAZ0Offset = 0;
		 theAZ1Offset = 0;
		 theAZ2Offset = 0;
		 theAZ3Offset = 0;
		 theLineOffset = 0;
		 theCcdRotCenterOffset = 0;
		 theCcdRotAngle = 0;
		 theCcdCenterXOffset = 0;
		 theCcdCenterYOffset = 0;
		 theCcdK1 = 0;
		 theCcdK2 = 0;
		 theCcdP1 = 0;
		 theCcdP2 = 0;
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
		  theCcdOffset      = computeParameterOffset(pos++);
		  theAX0Offset = computeParameterOffset(pos++);
		  theAX1Offset	   = computeParameterOffset(pos++);
		  theAX2Offset = computeParameterOffset(pos++);
		  theAX3Offset = computeParameterOffset(pos++);
		 theAY0Offset = computeParameterOffset(pos++);
		 theAY1Offset	   = computeParameterOffset(pos++);
		 theAY2Offset = computeParameterOffset(pos++);
		 theAY3Offset = computeParameterOffset(pos++);
		 theAY4Offset = computeParameterOffset(pos++);
		 theAY5Offset      = computeParameterOffset(pos++);
		 theAY6Offset      = computeParameterOffset(pos++);
		 theAZ0Offset = computeParameterOffset(pos++);
		 theAZ1Offset	   = computeParameterOffset(pos++);
		 theAZ2Offset = computeParameterOffset(pos++);
		 theAZ3Offset = computeParameterOffset(pos++);
		 theLineOffset      = computeParameterOffset(pos++);
		 theCcdRotCenterOffset = computeParameterOffset(pos++);
		 theCcdRotAngle = computeParameterOffset(pos++);
		 theCcdCenterXOffset = computeParameterOffset(pos++);
		 theCcdCenterYOffset = computeParameterOffset(pos++);
		 theCcdK1 = computeParameterOffset(pos++);
		 theCcdK2 = computeParameterOffset(pos++);
		 theCcdP1 = computeParameterOffset(pos++);
		 theCcdP2 = computeParameterOffset(pos++);
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

	  //updateApproximateModel();
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::updateModel(): returning..." << std::endl;
   }
   catch(...)
   {
      setErrorStatus(ossimErrorCodes::OSSIM_ERROR);
   }
}

void radiCbers04Model::initAdjustableParameters()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::initAdjustableParameters(): entering..." << std::endl;

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

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::initAdjustableParameters(): returning..." << std::endl;
}

void radiCbers04Model::loadSupportData()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "radiCbers04Model::loadSupportData(): entering..." << std::endl;

   //---
   // Check for good support data:
   //---
   if (!theSupportData)
   {
      setErrorStatus();
      ossimNotify(ossimNotifyLevel_FATAL) << "FATAL radiCbers04Model::loadSupportData(): Null SpotDimapSupportData pointer passed to"
                                          << " constructor! Aborting..." << std::endl;
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::loadSupportData(): returning..." << std::endl;
      return;
   }

   if (theSupportData->getErrorStatus() != ossimErrorCodes::OSSIM_OK)
   {
      setErrorStatus();
      ossimNotify(ossimNotifyLevel_FATAL) << "FATAL radiCbers04Model::loadSupportData(): Bad SpotDimapSupportData detected. Aborting..."
                                          << std::endl;
      if (traceExec()) ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::loadSupportData(): returning..." << std::endl;
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



   if (theSensorID.upcase().contains("P5M"))
   {
	   theFocalLen = 1009e-3;
	   theCcdStepLen = 6.5e-6;
	   theLineSamplingPeriod = 0.748e-3;
	   theCcdOffset = -8.9;
   }
   else if (theSensorID.upcase().contains("P10"))
   {
	   theFocalLen = 1009e-3;
	   theCcdStepLen = 13e-6;
	   theLineSamplingPeriod = 1.496e-3;
	   theCcdOffset = -4.5;
   }
   else if (theSensorID.upcase().contains("MUX"))
   {
	   theFocalLen = 500.71e-3;
	   theCcdStepLen = 13e-6;
	   theLineSamplingPeriod = 2.972e-3;
	   theCcdOffset = 0;
   }
   else if (theSensorID.upcase().contains("WFI"))
   {
	   theFocalLen = 150.43e-3;
	   theCcdStepLen = 13e-6;
	   theLineSamplingPeriod = 9.51e-3;
	   theCcdOffset = 0;
   }
   else if (theSensorID.upcase().contains("IRS"))
   {
	   theFocalLen = 150.43e-3;
	   theCcdStepLen = 13e-6;
	   theLineSamplingPeriod = 9.51e-3;
	   theCcdOffset = 0;
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

   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::loadSupportData(): returning..." << std::endl;
}

ossimObject* radiCbers04Model::dup() const
{
   return new radiCbers04Model(*this);
}

std::ostream& radiCbers04Model::print(std::ostream& out) const
{
   // Capture stream flags since we are going to mess with them.
   std::ios_base::fmtflags f = out.flags();

   out << "\nDump of radiCbers04Model at address " << (hex) << this
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
	<< "\n  theCcdOffset        = " << theCcdOffset
	<< "\n  theAX0Offset     = " << theAX0Offset
	<< "\n  theAX1Offset     = " << theAX1Offset
	<< "\n  theAX2Offset     = " << theAX2Offset
	<< "\n  theAX3Offset     = " << theAX3Offset
	   << "\n  theAY0Offset     = " << theAY0Offset
	   << "\n  theAY1Offset     = " << theAY1Offset
	   << "\n  theAY2Offset     = " << theAY2Offset
	   << "\n  theAY3Offset     = " << theAY3Offset
	   << "\n  theAY4Offset     = " << theAY4Offset
	   << "\n  theAY5Offset     = " << theAY5Offset
	   << "\n  theAY6Offset     = " << theAY6Offset
	   << "\n  theAZ0Offset     = " << theAZ0Offset
	   << "\n  theAZ1Offset     = " << theAZ1Offset
	   << "\n  theAZ2Offset     = " << theAZ2Offset
	   << "\n  theAZ3Offset     = " << theAZ3Offset
	   << "\n  theLineOffset     = " << theLineOffset 
	   << "\n  theRollOffset     = " << theRollOffset 
	   << "\n  thePitchOffset     = " << thePitchOffset 
	   << "\n  theYawOffset     = " << theYawOffset 
	   << "\n  theCcdRotCenterOffset     = " << theCcdRotCenterOffset 
	   << "\n  theCcdRotAngle     = " << theCcdRotAngle
	   << "\n  theCcdCenterXOffset     = " << theCcdCenterXOffset 
	   << "\n  theCcdCenterYOffset     = " << theCcdCenterYOffset 
	   << "\n  theCcdK1     = " << theCcdK1 
	   << "\n  theCcdK2     = " << theCcdK2 
	   << "\n  theCcdP1     = " << theCcdP1 
	   << "\n  theCcdP2     = " << theCcdP2 
       << "\n------------------------------------------------"
       << "\n  " << endl;

   // Set the flags back.
   out.flags(f);

   return ossimSensorModel::print(out);
}

bool radiCbers04Model::saveState(ossimKeywordlist& kwl,
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

bool radiCbers04Model::loadState(const ossimKeywordlist& kwl,
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

//void radiCbers04Model::imagingRay(const ossimDpt& image_point,
//							   ossimColumnVector3d& image_ray) const
//{
//
//	bool runtime_dbflag = 0;
//	NEWMAT::Matrix satToOrbit;
//	ossimDpt iPt = image_point;
//	//iPt.samp += theSpotSubImageOffset.samp;
//	//iPt.line += theSpotSubImageOffset.line;
//
//	double t_line = iPt.line + theLineOffset;
//	if (traceDebug() || runtime_dbflag)
//	{
//		ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::imagingRay():------------ BEGIN DEBUG PASS ---------------" << std::endl;
//		ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::imagingRay(): t_line = " << t_line << std::endl;
//	}
//
//	//
//	// 2. Interpolate ephemeris position and velocity (in ECF):
//	//
//	ossimEcefPoint  tempEcefPoint;
//	ossimEcefPoint  P_ecf;
//	theSupportData->getPositionEcf(t_line, P_ecf);
//	theSupportData->getVelocityEcf(t_line, tempEcefPoint);
//	ossimEcefVector V_ecf(tempEcefPoint.x(),
//		tempEcefPoint.y(),
//		tempEcefPoint.z());
//
//	double omega = 7.292115e-5;
//	ossimColumnVector3d vel_earth_rotation(0.0, 0.0, omega);
//	if (traceDebug() || runtime_dbflag)
//	{
//		ossimNotify(ossimNotifyLevel_DEBUG)
//			<< "DEBUG:\n\tP_ecf = " << P_ecf
//			<< "\n\t V_ecf = " << V_ecf << std::endl;
//	}
//
//
//	P_ecf = ossimEcefPoint(P_ecf.x(),
//		P_ecf.y(),
//		P_ecf.z());
//
//	//
//	// 3. Establish the look direction in Vehicle LSR space (S_sat).
//	//    ANGLES IN RADIANS
//	ossim_float64 phiX;// = computePhiX(iPt.samp);
//	ossim_float64 phiY;// = computePhiY(iPt.samp);
//	//ossimColumnVector3d u_sat(-tan(phiY), tan(phiX), -1.0);
//	//ossimColumnVector3d u_sat(phiY, phiX, 1.0);
//	computePhiXY(iPt.samp, phiX, phiY);
//	ossimColumnVector3d u_sat(phiY, phiX, -1.0);
//
//	// 
//	NEWMAT::Matrix ccdToSat;
//	getCcdToSatRotation(ccdToSat);
//
//	ossimColumnVector3d u_camera = ccdToSat*u_sat;
//	//image_ray = u_camera; return;
//	ossimColumnVector3d camera_offset(theXOffset, theYOffset, theZOffset);
//
//	//
//	// 4. Transform vehicle LSR space look direction vector to orbital LSR space
//	//    (S_orb):
//	//
//	computeSatToOrbRotation(satToOrbit, t_line);
//
//	ossimColumnVector3d u_orb = (satToOrbit*(u_camera+camera_offset)).unit();
//	//image_ray = u_orb; return;
//	if (traceDebug() || runtime_dbflag)
//	{
//		ossimNotify(ossimNotifyLevel_DEBUG)
//			<< "DEBUG:\n\t theSatToOrbRotation = " << satToOrbit
//			<< "\n\t u_orb = " << u_orb << endl;
//	}
//
//	//
//	// 5. Transform orbital LSR space look direction vector to ECF.
//	//
//	//   a. S_orb space Z-axis (Z_orb) is || to the ECF radial vector (P_ecf),
//	//   b. X_orb axis is computed as cross-product between velocity and radial,
//	//   c. Y_orb completes the orthogonal S_orb coordinate system.
//	//
//	//ossimColumnVector3d vel_earth_rotation(-omega * P_ecf.y(), omega * P_ecf.x(), 0.0);
//	//V_ecf = V_ecf - vel_earth_rotation.cross(ossimColumnVector3d(P_ecf.x(), P_ecf.y(), P_ecf.z()));
//	ossimColumnVector3d Z_orb (P_ecf.x(),
//		P_ecf.y(),
//		P_ecf.z());
//	//   Z_orb = Z_orb.unit();
//
//	//   ossimColumnVector3d X_orb = ossimColumnVector3d(V_ecf.x(),
//	//                                                   V_ecf.y(),
//	//												V_ecf.z()).cross(Z_orb).unit();
//	//ossimColumnVector3d Y_orb = Z_orb.cross(X_orb);
//	Z_orb = -Z_orb.unit();
//	ossimColumnVector3d X_orb = Z_orb.cross(ossimColumnVector3d(V_ecf.x(),
//		V_ecf.y(),
//		V_ecf.z())).unit();
//	ossimColumnVector3d Y_orb = X_orb.cross(Z_orb);
//
//	NEWMAT::Matrix orbToEcfRotation = NEWMAT::Matrix(3, 3);
//	orbToEcfRotation << X_orb[0] << Y_orb[0] << Z_orb[0]
//	<< X_orb[1] << Y_orb[1] << Z_orb[1]
//	<< X_orb[2] << Y_orb[2] << Z_orb[2];
//
//
//	double a = calcGAST(theRefImagingTime);
//	double sa = sin(a);
//	double ca = cos(a);
//	ossimDpt3d att(0.0, 0.0, 0.0);
//	att.x += thePitchOffset;
//	att.y += theRollOffset;
//	att.z += theYawOffset;
//	double cp = cos(att.x);
//	double sp = sin(att.x);
//	double cr = cos(att.y);
//	double sr = sin(att.y);
//	double cy = cos(att.z);
//	double sy = sin(att.z);
//	NEWMAT::Matrix j2000ToWGS84 = NEWMAT::Matrix(3,3);
//	j2000ToWGS84 << (cr*cy-sp*sr*sy) << (cr*sy+sp*sr*sy) << (-cp*sr)
//		<< (-cp*sy) << (cp*cy) << (sp)
//		<< (sr*cy+sp*cr*sy) << (sr*sy-sp*cr*cy) <<  cp*cr;
//
//	//NEWMAT::Matrix j2000ToWGS84 = NEWMAT::Matrix(3, 3);
//	//j2000ToWGS84 << ca << sa << 0.0
//	//<< -sa  << ca << 0.0
//	//<< 0.0 << 0.0 << 1;
//
//	//ossimColumnVector3d u_ecf  = (j2000ToWGS84*u_orb).unit();
//
//	//image_ray = (j2000ToWGS84*orbToEcfRotation*u_orb);
//	image_ray = (j2000ToWGS84*u_orb);
//}

void radiCbers04Model::imagingRay(const ossimDpt& image_point,
                                 ossimEcefRay&   image_ray) const
{
   bool runtime_dbflag = 0;
   NEWMAT::Matrix satToOrbit;
   ossimDpt iPt = image_point;
   iPt.x += theCcdCenterXOffset;
   iPt.y += theCcdCenterYOffset;

   double t_line;
   theSupportData->getLineTime(iPt.line, t_line);
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
   //double t_line = iPt.line + theLineOffset;
   if (traceDebug() || runtime_dbflag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::imagingRay():------------ BEGIN DEBUG PASS ---------------" << std::endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::imagingRay(): t_line = " << t_line << std::endl;
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
   //ossimColumnVector3d vel_earth_rotation(0.0, 0.0, omega);
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
   //ossimColumnVector3d u_sat(phiY, phiX, -1.0);
   ossimColumnVector3d u_sat(phiX, phiY, -1.0);
	//ossimColumnVector3d u_sat(-tan(phiY), tan(phiX), -1.0);
	//ossimColumnVector3d u_sat(phiY, phiX, 1.0);
	//ossim_float64 phiX;
	//ossim_float64 phiY;
	//computePhiXY(iPt.samp, phiX, phiY);
	//ossimColumnVector3d u_sat(phiX, phiY, -1.0);

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

	//ossimColumnVector3d u_orb = (satToOrbit*u_sat).unit();
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
	//ossimGpt origin(P_ecf);
	//double sin_lat = ossim::sind(origin.lat);
 //  double cos_lat = ossim::cosd(origin.lat);
 //  double sin_lon = ossim::sind(origin.lon);
 //  double cos_lon = ossim::cosd(origin.lon);
 //  
 //  ossimColumnVector3d E (-sin_lon,
 //                         cos_lon,
 //                         0.0);
 //  ossimColumnVector3d N (-sin_lat*cos_lon,
 //                         -sin_lat*sin_lon,
 //                         cos_lat);
 //  ossimColumnVector3d U (E.cross(N));

 //  //
 //  // Fill rotation matrix with these components, rotated about the Z axis
 //  // by the azimuth indicated:
 //  //
 //  double y_azimuth = (90 - 97.9486)*RAD_PER_DEG;
 //  NEWMAT::Matrix theLsrToEcefRotMatrix;
 //  if (std::abs(y_azimuth) > FLT_EPSILON)
 //  {
 //     double cos_azim = ossim::cosd(y_azimuth);
 //     double sin_azim = ossim::sind(y_azimuth);
 //     ossimColumnVector3d X (cos_azim*E - sin_azim*N);
 //     ossimColumnVector3d Y (sin_azim*E + cos_azim*N);
 //     ossimColumnVector3d Z (X.cross(Y));
 //     
 //     theLsrToEcefRotMatrix
 //        = ossimMatrix3x3::create(X[0], Y[0], Z[0],
 //                                 X[1], Y[1], Z[1],
 //                                 X[2], Y[2], Z[2]);
 //  }
 //  else
 //  {
 //     //***
 //     // No azimuth rotation, so simplify:
 //     //***
 //     theLsrToEcefRotMatrix = ossimMatrix3x3::create(E[0], N[0], U[0],
 //                                                    E[1], N[1], U[1],
 //                                                    E[2], N[2], U[2]);
 //  }

 //  ossimColumnVector3d u_ecf  = (theLsrToEcefRotMatrix*u_orb);


	//ossimColumnVector3d vel_earth_rotation(-omega * P_ecf.y(), -omega * P_ecf.x(), 0.0);
	ossimColumnVector3d vel_earth_rotation(0.0, 0.0, omega);
	//ossimColumnVector3d vel_earth_rotation(0.0, 0.0, 0.0);
	//V_ecf = ossimEcefVector(V_ecf.x()+vel_earth_rotation[0], V_ecf.y()+vel_earth_rotation[1], V_ecf.z()+vel_earth_rotation[2]);
	//V_ecf = V_ecf + vel_earth_rotation;
	vel_earth_rotation = vel_earth_rotation.cross(ossimColumnVector3d(P_ecf.x(), P_ecf.y(), P_ecf.z()));
	V_ecf = ossimEcefVector(V_ecf.x()+vel_earth_rotation[0], V_ecf.y()+vel_earth_rotation[1], V_ecf.z()+vel_earth_rotation[2]);
 //   ossimColumnVector3d Z_orb (P_ecf.x(),
 //                              P_ecf.y(),
 //                              P_ecf.z());
 ////   Z_orb = Z_orb.unit();

 ////   ossimColumnVector3d X_orb = ossimColumnVector3d(V_ecf.x(),
 ////                                                   V_ecf.y(),
	////												V_ecf.z()).cross(Z_orb).unit();
	////ossimColumnVector3d Y_orb = Z_orb.cross(X_orb);
	//Z_orb = -Z_orb.unit();
	//ossimColumnVector3d X_orb = Z_orb.cross(ossimColumnVector3d(V_ecf.x(),
	//	V_ecf.y(),
	//	V_ecf.z())).unit();
	//ossimColumnVector3d Y_orb = X_orb.cross(Z_orb);

	   ossimColumnVector3d Z_orb (P_ecf.x(),
	                              P_ecf.y(),
	                              P_ecf.z());
	Z_orb = Z_orb.unit();
	ossimColumnVector3d Y_orb = -ossimColumnVector3d(V_ecf.x(),
		V_ecf.y(),
		V_ecf.z()).cross(Z_orb).unit();
	ossimColumnVector3d X_orb = -Z_orb.cross(Y_orb);


	//ossimColumnVector3d Y_orb(V_ecf.x(),
	//	V_ecf.y(),
	//	V_ecf.z());
	//Y_orb = Y_orb.unit();
	//ossimColumnVector3d X_orb = ossimColumnVector3d(P_ecf.x(),
	//	P_ecf.y(),
	//	P_ecf.z()).cross(Y_orb).unit();
	//ossimColumnVector3d Z_orb = Y_orb.cross(X_orb).unit();

	//ossimColumnVector3d X_orb(V_ecf.x(),
	//	V_ecf.y(),
	//	V_ecf.z());
	//X_orb = X_orb.unit();
	//ossimColumnVector3d Y_orb = ossimColumnVector3d(P_ecf.x(),
	//	P_ecf.y(),
	//	P_ecf.z()).cross(X_orb).unit();
	//ossimColumnVector3d Z_orb = -Y_orb.cross(X_orb).unit();
	
    NEWMAT::Matrix orbToEcfRotation = NEWMAT::Matrix(3, 3);
    orbToEcfRotation << X_orb[0] << Y_orb[0] << Z_orb[0]
                        << X_orb[1] << Y_orb[1] << Z_orb[1]
						<< X_orb[2] << Y_orb[2] << Z_orb[2];


	double a = calcGAST(t_line);
	double sa = sin(a);
	double ca = cos(a);
	ossimDpt3d att(0.0, 0.0, 0.0);
	att.x += thePitchOffset;
	att.y += theRollOffset;
	att.z += theYawOffset;
	double cp = cos(att.x);
	double sp = sin(att.x);
	double cr = cos(att.y);
	double sr = sin(att.y);
	double cy = cos(att.z);
	double sy = sin(att.z);
	NEWMAT::Matrix j2000ToWGS84 = NEWMAT::Matrix(3,3);
	j2000ToWGS84 << (cr*cy-sp*sr*sy) << (cr*sy+sp*sr*sy) << (-cp*sr)
		<< (-cp*sy) << (cp*cy) << (sp)
		<< (sr*cy+sp*cr*sy) << (sr*sy-sp*cr*cy) <<  cp*cr;
	
	//j2000ToWGS84 << ca << sa << 0.0
	//<< -sa  << ca << 0.0
	//<< 0.0 << 0.0 << 1;
	
	//ossimColumnVector3d u_ecf  = (j2000ToWGS84*u_orb).unit();

	//ossimColumnVector3d u_ecf  = u_orb;
	//ossimColumnVector3d u_ecf = (j2000ToWGS84*u_orb);
	ossimColumnVector3d u_ecf = (j2000ToWGS84*orbToEcfRotation*u_orb);
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
          << "DEBUG Cbers04::imagingRay(): returning..." << std::endl;
    }
}

double radiCbers04Model::calcGAST(double t) const
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

void radiCbers04Model::lineSampleToWorld(const ossimDpt& image_point,
										 ossimGpt&       gpt) const
{
	ossimSensorModel::lineSampleToWorld(image_point, gpt);
	return;
}

//*****************************************************************************
//  METHOD: radiCbers04Model::worldToLineSample()
//  
//  Performs forward projection of ground point to image space.
//  
//*****************************************************************************
void radiCbers04Model::worldToLineSample(const ossimGpt& worldPoint,
                                         ossimDpt&       ip) const
{
	if (!!theApproximateModel)
	{
		theApproximateModel->worldToLineSample(worldPoint, ip);
		return;
	}
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
   // Note that this error message appears only if max count was reached while
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

void radiCbers04Model::lineSampleHeightToWorld(const ossimDpt& image_point,
                                              const ossim_float64& heightEllipsoid,
                                              ossimGpt& worldPoint) const
{
	//lineSampleToWorld(image_point, worldPoint);
	////NEWMAT::Matrix totalRotation;
	////ossimEcefPoint satPoint;
	////getExteriorParameters(image_point, totalRotation, satPoint);
	////ossimEcefPoint ecefPoint = collinearEquation_inverse(image_point, heightEllipsoid, totalRotation, satPoint);
	////worldPoint = ossimGpt(ecefPoint);
	//return;

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
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::lineSampleHeightToWorld(): returning..." << std::endl;
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

bool
radiCbers04Model::setupOptimizer(const ossimString& init_file)
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
radiCbers04Model::initFromMetadata(ossimQVProcSupportData* sd)
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
   theCcdOffset        = 0.0;
   theAY0Offset     = 0.0;
   theAY1Offset     = 0.0;
   theAY2Offset     = 0.0;
   theAY3Offset     = 0.0;
   theAY4Offset     = 0.0;
   theAY5Offset     = 0.0;
   theAY6Offset     = 0.0;
   theLineOffset     = 0.0;
   theRollOffset     = 0.0;
   thePitchOffset     = 0.0;
   theYawOffset     = 0.0;


   //---
   // Instantiate the support data classes after establishing the filenames:
   //---
   loadSupportData();
   if (getErrorStatus() != ossimErrorCodes::OSSIM_OK)
   {
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG radiCbers04Model::initFromMetadata(dimap_file): returning with error..." << std::endl;
      return false;
   }


   //---
   // initialize remaining data members:
   //---
   initAdjustableParameters();
   updateModel();
   return true;
}


void radiCbers04Model::setImageRect(const ossimDrect& roi)
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
//	radiCbers04Model::getForwardDeriv(int iAtt, int iComponent, const ossimGpt& gpos, double hdelta)
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
//	radiCbers04Model::buildNormalEquation(const ossimTieGptSet& tieSet,
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
//	radiCbers04Model::optimizeFit(const ossimTieGptSet& tieSet, double* /* targetVariance */)
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
}