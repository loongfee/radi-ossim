#include <cstdlib>
#include <ossim/projection/ossimLandSatModel.h>
RTTI_DEF1(ossimLandSatModel, "ossimLandSatModel", ossimSensorModel);
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/support_data/ossimFfL7.h>
#include <ossim/support_data/ossimFfL5.h>
#include <ossim/projection/ossimSpaceObliqueMercatorProjection.h>
#include <ossim/projection/ossimUtmProjection.h>
#include <ossim/projection/ossimTransMercatorProjection.h>
#include <ossim/projection/ossimMapProjection.h>
#include <ossim/base/ossimLsrPoint.h>
#include <ossim/base/ossimLsrRay.h>
#include <ossim/base/ossimLsrSpace.h>
#include <stdio.h>
#include <fstream>
#include <ossim/base/ossimTrace.h>
static ossimTrace traceExec  ("ossimLandSatModel:exec");
static ossimTrace traceDebug ("ossimLandSatModel:debug");
static const int    MODEL_VERSION_NUMBER   = 2;
static const char*  PROJECTION_TYPE_KW     = "theProjectionType";
static const char*  MAP_ZONE_KW            = "theMapZone";
static const char*  MAP_OFFSET_X_KW        = "theMapOffset.x";
static const char*  MAP_OFFSET_Y_KW        = "theMapOffset.y";
static const char*  WRS_PATH_NUMBER_KW     = "theWrsPathNumber";
static const char*  ROW_NUMBER_KW          = "theWrsRowNumber";
static const char*  ILLUM_AZIMUTH_KW       = "theIllumAzimuth";
static const char*  ILLUM_ELEVATION_KW     = "theIllumElevation";
static const char*  MERIDIANAL_ANGLE_KW    = "theMeridianalAngle";
static const char*  ORBIT_ALTITUDE_KW      = "theOrbitAltitude";
static const char*  ORBIT_INCLINATION_KW   = "theOrbitInclination";
static const char*  MAP_AZIM_ANGLE_KW      = "theMapAzimAngle";
static const char*  MAP_2Ic_ROT_ANGLE_KW   = "theMap2IcRotAngle";
static const char*  INTRACK_OFFSET_KW      = "theIntrackOffset";
static const char*  CRTRACK_OFFSET_KW      = "theCrtrackOffset";
static const char*  LINE_GSD_CORR_KW       = "theLineGsdCorr";
static const char*  SAMP_GSD_CORR_KW       = "theSampGsdCorr";
static const char*  ROLL_OFFSET_KW         = "theRollOffset";
static const char*  YAW_OFFSET_KW          = "theYawOffset";
static const char*  YAW_RATE_KW            = "theYawRate";
static const char*  MAP_ROTATION_KW        = "theMapRotation";
static const char*  MAP_SCALE_KW        = "theMapScale";
static const char*  MAP_CENTER_LONGITUDE_KW        = "theMapCenterLongitude";
static const char*  MAP_OFFSET_LONGITUDE_KW        = "theMapoffsetX";
static const char*  MAP_OFFSET_LATITUDE_KW        = "theMapoffsetY";
static const char*  MAP_SEMI_MAJOR_KW        = "theMapSemiMajorX";
static const char*  MAP_SEMI_MINOR_KW        = "theMapSemiMinorY";
static const double   GEODETIC_2_GEOCENTRIC_FACTOR  = 1.00674;
static const double   L7_ORBIT_ALTITUDE  = 705300.0;
static const double   L7_ORBIT_INCLINATION  = 98.22;
static const double   L7_NOMINAL_POS_ERROR  = 200.0;
static const double   L5_ORBIT_ALTITUDE  = 705300.0;
static const double   L5_ORBIT_INCLINATION  = 98.22;
static const double   L5_NOMINAL_POS_ERROR  = 12000.0; //arbitrary : to be fixed
static const char* PARAM_NAMES[] ={"intrack_offset",
                                         "crtrack_offset",
                                         "line_gsd_corr",
                                         "samp_gsd_corr",
                                         "roll_offset",
                                         "yaw_offset",
                                         "yaw_rate",
                                         "map_rotation"};
static const char* PARAM_UNITS[] ={"meters",
                                         "meters",
                                         "meters",
                                         "meters",
                                         "degrees",
                                         "degrees",
                                         "seconds",
                                         "degrees"};
static const char* PROJ_TYPE[] = { "UNKNOWN_PROJECTION",
                                         "UTM_MAP",
                                         "UTM_ORBIT",
                                         "SOM_MAP",
                                         "SOM_ORBIT",
										"TM_MAP",
										"TM_ORBIT"};
ossimLandSatModel::ossimLandSatModel()
   :
   ossimSensorModel(),
   theIntrackOffset     (0.0),
   theCrtrackOffset     (0.0),
   theLineGsdCorr       (0.0),   
   theSampGsdCorr       (0.0),
   theRollOffset        (0.0),
   theYawOffset         (0.0),
   theYawRate           (0.0),
   theMapRotation       (0.0)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel: entering..." << std::endl;
   initAdjustableParameters();
   
   if (traceExec()) ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel: returning..." << std::endl;
}
ossimLandSatModel::ossimLandSatModel(const ossimFfL7& head)
   :
   ossimSensorModel(),
   theIntrackOffset     (0.0),
   theCrtrackOffset     (0.0),
   theLineGsdCorr       (0.0),   
   theSampGsdCorr       (0.0),
   theRollOffset        (0.0),
   theYawOffset         (0.0),
   theYawRate           (0.0),
   theMapRotation       (0.0)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel(head): entering..." << std::endl;   
   initFromHeader(head);
   if (traceDebug()) ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel(head): Exited..." << std::endl;
}
ossimLandSatModel::ossimLandSatModel(const ossimFilename& init_file)
 
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel(init_file): entering..." << std::endl;
   setupOptimizer(init_file);
   if (traceExec()) ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel(init_file): Exited..." << std::endl;
}
ossimLandSatModel::ossimLandSatModel(const ossimKeywordlist& geom_kwl)
   :
   ossimSensorModel(),
   theIntrackOffset     (0.0),
   theCrtrackOffset     (0.0),
   theLineGsdCorr       (0.0),   
   theSampGsdCorr       (0.0),
   theRollOffset        (0.0),
   theYawOffset         (0.0),
   theYawRate           (0.0),
   theMapRotation       (0.0)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel(geom_kwl): entering..." << std::endl;
   initAdjustableParameters();
   loadState(geom_kwl);
   if (traceExec()) ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel(geom_kwl): Exited..." << std::endl;
}
ossimLandSatModel::ossimLandSatModel(const ossimLandSatModel& rhs)
   :
   ossimSensorModel   (rhs),
   theIllumAzimuth    (rhs.theIllumAzimuth),
   theIllumElevation  (rhs.theIllumElevation),
   theOrbitAltitude   (rhs.theOrbitAltitude),
   theOrbitInclination(rhs.theOrbitInclination),
   theMapZone         (rhs.theMapZone),
   theMapOffset       (rhs.theMapOffset),
   theWrsPathNumber   (rhs.theWrsPathNumber),
   theWrsRowNumber    (rhs.theWrsRowNumber),
   theMeridianalAngle (rhs.theMeridianalAngle),
   thePositionError   (rhs.thePositionError),
   theProjectionType  (rhs.theProjectionType),
   theMapProjection   (rhs.theMapProjection.valid()?
                       (ossimMapProjection*)rhs.theMapProjection->dup():
                       (ossimMapProjection*)0),
   theMapAzimAngle    (rhs.theMapAzimAngle),
   theMapAzimCos      (rhs.theMapAzimCos),
   theMapAzimSin      (rhs.theMapAzimSin),
   theMap2IcRotAngle  (rhs.theMap2IcRotAngle),
   theMap2IcRotCos    (rhs.theMap2IcRotCos),
   theMap2IcRotSin    (rhs.theMap2IcRotSin),
   theIntrackOffset   (rhs.theIntrackOffset),
   theCrtrackOffset   (rhs.theCrtrackOffset),
   theLineGsdCorr     (rhs.theLineGsdCorr),
   theSampGsdCorr     (rhs.theSampGsdCorr),
   theRollOffset      (rhs.theRollOffset),
   theYawOffset       (rhs.theYawOffset),
   theYawRate         (rhs.theYawRate),
   theMapRotation     (rhs.theMapRotation),
   theRollRotMat      (rhs.theRollRotMat)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel(rhs): entering..." << std::endl;
   
   initAdjustableParameters();
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::ossimLandSatModel(rhs): returning..." << std::endl;
}
ossimObject* ossimLandSatModel::dup()const
{
   return new ossimLandSatModel(*this);
}
ossimLandSatModel::~ossimLandSatModel()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::~ossimLandSatModel: entering..." << std::endl;
   theMapProjection = 0;
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::~ossimLandSatModel: returning..." << std::endl;
}
void ossimLandSatModel::initFromHeader(const ossimFfL7& head)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::initFromHeader: entering..." << std::endl;
   
   theRefGndPt         = head.theCenterGP;
   theRefImgPt         = head.theCenterImagePoint;
   theImageSize.x      = head.thePixelsPerLine;
   theImageSize.y      = head.theLinesPerBand;
   theImageID          = head.theRequestNumber;
   theSensorID		   = head.theSatName;
   theSensorDATE	   = head.theAcquisitionDate;
   theImageClipRect    = ossimDrect(0, 0, theImageSize.x-1, theImageSize.y-1);
   theGSD.samp         = head.theGsd;
   theGSD.line         = head.theGsd;
   theIllumAzimuth     = head.theSunAzimuth;
   theIllumElevation   = head.theSunElevation;
   theMapZone          = head.theUsgsMapZone;
   theWrsPathNumber    = head.thePathNumber;
   theWrsRowNumber     = head.theRowNumber;
   theRollOffset       = head.theOffNadirAngle;
   theMeanGSD          = head.theGsd;
   m_scale=head.theProjectionParams[2];
	m_centerLongitude=(int)(head.theProjectionParams[4]+0.01)*1.0;
	m_theMapoffsetX=head.theProjectionParams[6];
	m_theMapoffsetY=head.theProjectionParams[5];
	semi_major=head.theProjectionParams[0];
	semi_minor=head.theProjectionParams[1];
   
	//int longcenter=(int)(head.theProjectionParams[4]+0.5);
	//theProjectionParams[4]=(double)longcenter;
   ossimString satname(head.theSatName);
   if (satname.contains("7"))
   {
      theOrbitAltitude    = L7_ORBIT_ALTITUDE;
      theOrbitInclination = L7_ORBIT_INCLINATION;
      theNominalPosError  = L7_NOMINAL_POS_ERROR;
   } else if (satname.contains("5"))
   {
      theOrbitAltitude    = L5_ORBIT_ALTITUDE;
      theOrbitInclination = L5_ORBIT_INCLINATION;
      theNominalPosError  = L5_NOMINAL_POS_ERROR;
   } else {
      theErrorStatus = 1; //MODEL_ERROR
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_WARN) << "WARNING ossimLandSatModel::initFromHeader: " << "Unknown satellite name : " << satname << std::endl;
      }
   }   
   
   
   double phi_c = ossim::atand(ossim::tand(theRefGndPt.lat)/
                               GEODETIC_2_GEOCENTRIC_FACTOR);
   double cos_phi_c   = ossim::cosd(phi_c);
   theMeridianalAngle = -ossim::asind(ossim::cosd(theOrbitInclination) / cos_phi_c);
   theMapAzimAngle = head.theOrientationAngle;
   ossimDpt v[4]; // temporarily holds vertices for ground polygon
   v[0] = head.theUL_Corner;
   v[1] = head.theUR_Corner;
   v[2] = head.theLR_Corner;
   v[3] = head.theLL_Corner;
   theBoundGndPolygon = ossimPolygon(4, v);
   ossimString orient_type = head.theProductType;
   ossimString proj_type   = head.theMapProjectionName;
   proj_type.trim();
   if (proj_type.contains("SOM"))
   {
      if (orient_type.contains("ORBIT"))
         theProjectionType  = SOM_ORBIT;
      else
         theProjectionType = SOM_MAP;
      theMapAzimAngle -= 90.0;
   }
   else if (proj_type.contains("UTM"))
   {
      if (orient_type.contains("ORBIT"))
         theProjectionType  = UTM_ORBIT;
      else
         theProjectionType = UTM_MAP;
   }
      else if (proj_type.contains("TM"))
   {
      if (orient_type.contains("ORBIT"))
         theProjectionType  = TM_ORBIT;
      else
         theProjectionType = TM_MAP;
   }
   else
   {
      theErrorStatus = 1; //MODEL_ERROR
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_WARN) << "WARNING ossimLandSatModel::initFromHeader: "
                                             << "Unknown projection/orientation type." << std::endl;
      }
      return;
   }
   initMapProjection();
      
   theMapOffset = theMapProjection->forward(head.theUL_Corner);
   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "DEBUG ossimLandSatModel::initFromHeader:"
         << "\ntheMapProjection:\n";
      theMapProjection->print(ossimNotify(ossimNotifyLevel_DEBUG));
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "\nHeader upper left ground point:  " << head.theUL_Corner
         << std::endl;
   }
   initAdjustableParameters();
   updateModel();
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::initFromHeader: returning..." << std::endl;
}
void ossimLandSatModel::lineSampleHeightToWorld(const ossimDpt& image_point,
                                                const double&   height,
                                                ossimGpt&       gpt) const
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::lineSampleHeightToWorld: entering..." << std::endl;
#if 0
   if (!insideImage(image_point))
   {
      gpt = extrapolate(image_point, height);
      if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::lineSampleHeightToWorld: returning..." << std::endl;
      return;
   }
#endif
   
   ossimEcefRay imaging_ray;
   imagingRay(image_point, imaging_ray);
   //ossimEcefPoint Pecf (imaging_ray.intersectAboveEarthEllipsoid(height));
   //gpt = ossimGpt(Pecf);
   if (m_proj==NULL)  {
	   ossimEcefPoint Pecf (imaging_ray.intersectAboveEarthEllipsoid(height));
		gpt = ossimGpt(Pecf);
   }
   else
   {
	   ossimEcefPoint Pecf (imaging_ray.intersectAboveEarthEllipsoid(height,m_proj->getDatum()));
   gpt = ossimGpt(Pecf,m_proj->getDatum());
   }

}
   
   
void ossimLandSatModel::imagingRay(const ossimDpt& inImgPt,
                                   ossimEcefRay&   image_ray) const
{
#if 0
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "ossimLandSatModel::imagingRay: entering..." << std::endl;
   bool debug_flag = false; // setable by interactive debugger
   if (traceDebug() || debug_flag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "inImgPt = " << inImgPt << std::endl;
   }
#endif
   
   ossimDpt rot_img_pt(-inImgPt.line*theMapAzimSin+inImgPt.samp*theMapAzimCos,
                        inImgPt.line*theMapAzimCos+inImgPt.samp*theMapAzimSin);
   ossimDpt map_point 
      (theMapOffset.x + rot_img_pt.samp*(theGSD.samp+theSampGsdCorr), 
       theMapOffset.y - rot_img_pt.line*(theGSD.line+theLineGsdCorr));
   ossimGpt inGndPt (theMapProjection->inverse(map_point));
#if 0
   if (traceDebug() || debug_flag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t theMapOffset="<<theMapOffset<<endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t rot_img_pt="<<rot_img_pt<<endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t image point map_point="<<map_point<<endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t inGndPt="<<inGndPt<<endl;
   }
#endif
   ossimDpt offInMapPt (inImgPt - theRefImgPt);
   ossimDpt icInPt
      (offInMapPt.line*theMap2IcRotSin + offInMapPt.samp*theMap2IcRotCos,
       offInMapPt.line*theMap2IcRotCos - offInMapPt.samp*theMap2IcRotSin);
#if 0
   if (traceDebug() || debug_flag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t offInMapPt="<<offInMapPt<<endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t icInPt="<<icInPt<<endl;
   }
#endif
   
   ossimDpt icNdrPt (0.0, icInPt.line);
   ossimDpt offNdrMapPt
      (-icNdrPt.line*theMap2IcRotSin + icNdrPt.samp*theMap2IcRotCos,
        icNdrPt.line*theMap2IcRotCos + icNdrPt.samp*theMap2IcRotSin);
   ossimDpt ndrMapPt(offNdrMapPt + theRefImgPt);
   ossimDpt rotNdrMapPt
      (-ndrMapPt.line*theMapAzimSin + ndrMapPt.samp*theMapAzimCos,
       ndrMapPt.line*theMapAzimCos + ndrMapPt.samp*theMapAzimSin);
#if 0
   if (traceDebug() || debug_flag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t icNdrPt="<<icNdrPt<<endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t offNdrMapPt="<<offNdrMapPt<<endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t ndrMapPt="<<ndrMapPt<<endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t rotNdrMapPt="<<rotNdrMapPt<<endl;
   }
#endif
    map_point.y =theMapOffset.y-rotNdrMapPt.y*(theGSD.y+theLineGsdCorr);//theSampGsdCorr);
   if ((theProjectionType == SOM_ORBIT) || (theProjectionType == SOM_MAP))
      map_point.x = theMapOffset.x+rotNdrMapPt.y*(theGSD.y+theLineGsdCorr);
   else
      map_point.x = theMapOffset.x+rotNdrMapPt.x*(theGSD.x+theSampGsdCorr);
   ossimGpt vehiclePlhPos(theMapProjection->inverse(map_point));
   vehiclePlhPos.hgt = theOrbitAltitude;
#if 0
   if (traceDebug() || debug_flag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t map_point="<<map_point<<endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t vehiclePlhPos="<<vehiclePlhPos<<endl;
   }
#endif
   
   ossimLsrSpace icrSpace (vehiclePlhPos, theMeridianalAngle-90.0);
#if 0
   if (traceDebug() || debug_flag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t icrSpace="<<icrSpace<<endl;
   }
#endif
   ossimLsrPoint lsrInPt (inGndPt, icrSpace);
   ossimLsrPoint vehicleLsrPos (0.0, 0.0, 0.0, icrSpace);
   ossimLsrRay   initLsrImgRay (vehicleLsrPos, lsrInPt);
#if 0
   if (traceDebug() || debug_flag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t initLsrImgRay="<<initLsrImgRay<<endl;
   }
#endif
   double cos, sin;
   double norm_line = inImgPt.line/theImageSize.line;
   double yaw = theYawOffset + theYawRate*norm_line;
   cos = ossim::cosd(yaw);
   sin = ossim::sind(yaw);
   NEWMAT::Matrix T_yaw = ossimMatrix3x3::create( cos,-sin, 0.0,
                                                  sin, cos, 0.0,
                                                  0.0, 0.0, 1.0);
   NEWMAT::Matrix attRotMat = T_yaw * theRollRotMat;
   ossimLsrVector adjLsrImgRayDir (attRotMat*initLsrImgRay.direction().data(),
                                   icrSpace);
   ossimLsrPoint  adjLsrImgRayOrg (theIntrackOffset,
                                   theCrtrackOffset,
                                   0.0,  // no radial adjustment of position
                                   icrSpace);
   ossimLsrRay adjLsrImgRay (adjLsrImgRayOrg, adjLsrImgRayDir);
   image_ray = ossimEcefRay(adjLsrImgRay);
#if 0
   if (traceDebug() || debug_flag)
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t adjLsrImgRay="<<adjLsrImgRay<<endl;
      ossimNotify(ossimNotifyLevel_DEBUG) << "\t image_ray="<<image_ray<<endl;
   }
   if (traceExec())
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::imagingRay: Returning..." << std::endl;
   }
#endif
}
std::ostream& ossimLandSatModel::print(std::ostream& os) const
{
   os << "\nDump of ossimLandSatModel object at "
      << hex << this << ":\n"
      << "\nLandSatModel -- Dump of all data members: "
      << "\n         theImageID: " << theImageID.chars()
      << "\n       theImageSize: " << theImageSize
      << "\n        theRefImgPt: " << theRefImgPt
      << "\n        theRefGndPt: " << theRefGndPt
      << "\n        theGSD.line: " << theGSD.line
      << "\n        theGSD.samp: " << theGSD.samp
      << "\n  theProjectionType: " << PROJ_TYPE[theProjectionType]
      << "\n         theMapZone: " << theMapZone
      << "\n       theMapOffset: " << theMapOffset
      << "\n   theWrsPathNumber: " << theWrsPathNumber
      << "\n    theWrsRowNumber: " << theWrsRowNumber
      << "\n    theIllumAzimuth: " << theIllumAzimuth
      << "\n  theIllumElevation: " << theIllumElevation
      << "\n   thePositionError: " << thePositionError
      << "\n theMeridianalAngle: " << theMeridianalAngle
      << "\n   theOrbitAltitude: " << theOrbitAltitude
      << "\ntheOrbitInclination: " << theOrbitInclination
      << "\n    theMapAzimAngle: " << theMapAzimAngle
      << "\n  theMap2IcRotAngle: " << theMap2IcRotAngle
      << "\n   theIntrackOffset: " << theIntrackOffset
      << "\n   theCrtrackOffset: " << theCrtrackOffset
      << "\n     theLineGsdCorr: " << theLineGsdCorr
      << "\n     theSampGsdCorr: " << theSampGsdCorr
      << "\n      theRollOffset: " << theRollOffset
      << "\n       theYawOffset: " << theYawOffset
      << "\n         theYawRate: " << theYawRate
      << "\n     theMapRotation: " << theMapRotation
	  << "\n         m_scale: " << m_scale
      << "\n     m_centerLongitude: " << m_centerLongitude
	  << "\n         m_theMapoffsetX: " << m_theMapoffsetX
      << "\n     m_theMapoffsetY: " << m_theMapoffsetY
	  << "\n         semi_major: " << semi_major
      << "\n     semi_minor: " << semi_minor
      << endl;
   return ossimSensorModel::print(os);
}
bool ossimLandSatModel::saveState(ossimKeywordlist& kwl,
                              const char* prefix) const
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::saveState: entering..." << std::endl;
   kwl.add(prefix, ossimKeywordNames::TYPE_KW, TYPE_NAME(this));
   ossimSensorModel::saveState(kwl, prefix);
   kwl.add(prefix, PROJECTION_TYPE_KW,   theProjectionType, true);
   kwl.add(prefix, MAP_ZONE_KW,          theMapZone, true);
   kwl.add(prefix, MAP_OFFSET_X_KW,      theMapOffset.x, true);
   kwl.add(prefix, MAP_OFFSET_Y_KW,      theMapOffset.y, true);
   kwl.add(prefix, WRS_PATH_NUMBER_KW,   theWrsPathNumber, true);
   kwl.add(prefix, ROW_NUMBER_KW,        theWrsRowNumber, true);
   kwl.add(prefix, ILLUM_AZIMUTH_KW,     theIllumAzimuth, true);
   kwl.add(prefix, ILLUM_ELEVATION_KW,   theIllumElevation, true);
   kwl.add(prefix, MERIDIANAL_ANGLE_KW,  theMeridianalAngle, true);
   kwl.add(prefix, ORBIT_ALTITUDE_KW,    theOrbitAltitude, true);
   kwl.add(prefix, ORBIT_INCLINATION_KW, theOrbitInclination, true);
   kwl.add(prefix, MAP_AZIM_ANGLE_KW,    theMapAzimAngle, true);
   kwl.add(prefix, MAP_2Ic_ROT_ANGLE_KW, theMap2IcRotAngle, true);
   kwl.add(prefix, MAP_SCALE_KW,    m_scale, true);
   kwl.add(prefix, MAP_CENTER_LONGITUDE_KW, m_centerLongitude, true);
   kwl.add(prefix, MAP_OFFSET_LONGITUDE_KW,    m_theMapoffsetX, true);
   kwl.add(prefix, MAP_OFFSET_LATITUDE_KW, m_theMapoffsetY, true);
   kwl.add(prefix, MAP_SEMI_MAJOR_KW,    semi_major, true);
   kwl.add(prefix, MAP_SEMI_MINOR_KW, semi_minor, true);
   
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::saveState: returning..." << std::endl;
   return true;
}
bool ossimLandSatModel::loadState(const ossimKeywordlist& kwl,
                                  const char* prefix) 
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::loadState: entering..." << std::endl;
   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::loadState:"
                                          << "\nInput kwl:  " << kwl
                                          << std::endl;
   }
   const char* value = NULL;
   const char* keyword =NULL;
   bool success;
   value = kwl.find(prefix, ossimKeywordNames::TYPE_KW);
   if (!value || (strcmp(value, TYPE_NAME(this)))) 
     {
       theErrorStatus = 1;
       return false;
     }
   if(getNumberOfAdjustableParameters() != NUM_ADJUSTABLE_PARAMS)
     {
       initAdjustableParameters();
     }
   
   success = ossimSensorModel::loadState(kwl, prefix);
   if (!success) 
     {
       theErrorStatus++;
       return false;
     }
 
   keyword = PROJECTION_TYPE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theProjectionType = (ProjectionType) atoi(value);
 
   keyword = MAP_ZONE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theMapZone = atoi(value);
   if(theMapZone==0) {
   
  /* static const char*  MAP_SCALE_KW        = "theMapScale";
static const char*  MAP_CENTER_LONGITUDE_KW        = "theMapCenterLongitude";
static const char*  MAP_OFFSET_LONGITUDE_KW        = "theMapoffsetX";
static const char*  MAP_OFFSET_LATITUDE_KW        = "theMapoffsetY";
   
      double			m_scale;
   double			m_centerLongitude;
   double			m_theMapoffsetX;
   double		    m_theMapoffsetY;*/
   keyword = MAP_SCALE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   m_scale = atof(value);
   keyword = MAP_CENTER_LONGITUDE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   m_centerLongitude = atof(value);
      keyword = MAP_OFFSET_LONGITUDE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   m_theMapoffsetX = atof(value);
      keyword = MAP_OFFSET_LATITUDE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   m_theMapoffsetY = atof(value);
         keyword = MAP_SEMI_MAJOR_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   semi_major = atof(value);
         keyword = MAP_SEMI_MINOR_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   semi_minor= atof(value);
   
   }
 
   keyword = MAP_OFFSET_X_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theMapOffset.x = atof(value);
 
   keyword = MAP_OFFSET_Y_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theMapOffset.y = atof(value);
 
   keyword = WRS_PATH_NUMBER_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theWrsPathNumber = atoi(value);
   keyword = ROW_NUMBER_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theWrsRowNumber = atoi(value);
 
   keyword = ILLUM_AZIMUTH_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theIllumAzimuth = atof(value);
 
   keyword = ILLUM_ELEVATION_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theIllumElevation = atof(value);
 
   keyword = MERIDIANAL_ANGLE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theMeridianalAngle = atof(value);
 
   keyword = ORBIT_ALTITUDE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theOrbitAltitude = atof(value);
   
   keyword = ORBIT_INCLINATION_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theOrbitInclination = atof(value);
   
   keyword = MAP_AZIM_ANGLE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theMapAzimAngle = atof(value);
 
   keyword = MAP_2Ic_ROT_ANGLE_KW;
   value = kwl.find(prefix, keyword);
   if (!value)
     {
       theErrorStatus++;
       return false;
     }
   theMap2IcRotAngle = atof(value);
   theIntrackOffset = 0.0;
   theCrtrackOffset = 0.0;
   theLineGsdCorr   = 0.0;
   theSampGsdCorr   = 0.0;
   theRollOffset    = 0.0;
   theYawOffset     = 0.0;
   theYawRate       = 0.0;
   theMapRotation   = 0.0;
   
   
   
   
   initMapProjection();
   updateModel();
   
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::loadState: returning..." << std::endl;
   return true;
}
void ossimLandSatModel::writeGeomTemplate(ostream& os)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::writeGeomTemplate: entering..." << std::endl;
   os <<
      "//**************************************************************\n"
      "// Template for LandSat model keywordlist\n"
      "//**************************************************************\n"
      << ossimKeywordNames::TYPE_KW << ": " << "ossimLandSatModel" << endl;
   ossimSensorModel::writeGeomTemplate(os);
   
   os << "//\n"
      << "// Derived-class ossimLandSatModel Keywords:\n"
      << "//\n"
      << PROJECTION_TYPE_KW     << ": <float>\n"
      << MAP_ZONE_KW            << ": <float>\n"
      << MAP_OFFSET_X_KW        << ": <float>\n"
      << MAP_OFFSET_Y_KW        << ": <float>\n"
      << WRS_PATH_NUMBER_KW     << ": <float>\n"
      << ROW_NUMBER_KW          << ": <float>\n"
      << ILLUM_AZIMUTH_KW       << ": <float>\n"
      << ILLUM_ELEVATION_KW     << ": <float>\n"
      << MERIDIANAL_ANGLE_KW    << ": <float>\n"
      << ORBIT_ALTITUDE_KW      << ": <float>\n"
      << ORBIT_INCLINATION_KW   << ": <float>\n"
      << MAP_AZIM_ANGLE_KW      << ": <float>\n"
      << MAP_2Ic_ROT_ANGLE_KW   << ": <float>\n"
      << INTRACK_OFFSET_KW      << ": <float> [optional]\n"
      << CRTRACK_OFFSET_KW      << ": <float> [optional]\n"
      << LINE_GSD_CORR_KW       << ": <float> [optional]\n"
      << SAMP_GSD_CORR_KW       << ": <float> [optional]\n"
      << ROLL_OFFSET_KW         << ": <float> [optional]\n"
      << YAW_OFFSET_KW          << ": <float> [optional]\n"
      << YAW_RATE_KW            << ": <float> [optional]\n"
      << MAP_ROTATION_KW        << ": <float> [optional]\n"
      << endl;
   os << "\n" <<endl;
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::writeGeomTemplate: returning..." << std::endl;
   return;
}
void ossimLandSatModel::initMapProjection()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) <<  "DEBUG ossimLandSatModel::writeGeomTemplate: entering... " << std::endl;
   theMapProjection = 0;   
   if ((theProjectionType == SOM_ORBIT) || (theProjectionType == SOM_MAP))
   {
      theMapProjection = new ossimSpaceObliqueMercatorProjection(
         ossimSpaceObliqueMercatorProjection::SOM_TYPE_LANDSAT_7,
         (double)theWrsPathNumber);
   }
   else if (theProjectionType == TM_MAP || theProjectionType == TM_ORBIT)
   {
	  // ossimGpt origintm(theProjectionParams[3],theProjectionParams[4]);
	//   ossimGpt origintm(0.0,0.0);//ww0712
	  // theMapZone=(int)((m_centerLongitude+3)/6);
	   ossimTransMercatorProjection* transMerc1 = new ossimTransMercatorProjection(ossimEllipsoid(semi_major, semi_minor),(int)((m_centerLongitude+3)/6));
    transMerc1->setParameters(m_theMapoffsetX,m_theMapoffsetY,m_scale);
	transMerc1->setZone((int)((m_centerLongitude+3)/6));
	transMerc1->update();
	  theMapProjection = transMerc1;
     }
   else 
   {
      ossimUtmProjection* utm   = new ossimUtmProjection(theMapZone);
      theMapProjection = utm;
      if(theRefGndPt.latd() < 0.0)
      {
         utm->setHemisphere('S');
      }
      else
      {
         utm->setHemisphere('N');
      }
   }
   theMap2IcRotAngle = theMeridianalAngle + theMapAzimAngle;
   theMap2IcRotCos   = ossim::cosd(theMap2IcRotAngle);
   theMap2IcRotSin   = ossim::sind(theMap2IcRotAngle);
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) <<  "DEBUG ossimLandSatModel::writeGeomTemplate: returning... " << std::endl;
   return;
}
void  ossimLandSatModel::updateModel()
{
  theIntrackOffset = computeParameterOffset(INTRACK_OFFSET);
  theCrtrackOffset = computeParameterOffset(CRTRACK_OFFSET);
  theLineGsdCorr   = computeParameterOffset(LINE_GSD_CORR);
  theSampGsdCorr   = computeParameterOffset(SAMP_GSD_CORR);
  theRollOffset    = computeParameterOffset(ROLL_OFFSET);
  theYawOffset     = computeParameterOffset(YAW_OFFSET);
  theYawRate       = computeParameterOffset(YAW_RATE);
  theMapRotation   = computeParameterOffset(MAP_ROTATION);
   if (theProjectionType == UTM_ORBIT)
   {
      theMapAzimCos = ossim::cosd(-theMapAzimAngle + theMapRotation);
      theMapAzimSin = ossim::sind(-theMapAzimAngle + theMapRotation);
   }
   else if (theProjectionType == TM_ORBIT)
   {
	 theMapAzimCos = ossim::cosd(-theMapAzimAngle + theMapRotation);
     theMapAzimSin = ossim::sind(-theMapAzimAngle + theMapRotation);
   
   }
   else
   {
      theMapAzimCos = ossim::cosd(theMapAzimAngle + theMapRotation);
      theMapAzimSin = ossim::sind(theMapAzimAngle + theMapRotation);
   }
   double cos = ossim::cosd(theRollOffset);
   double sin = ossim::sind(theRollOffset);
   theRollRotMat = ossimMatrix3x3::create( 1.0, 0.0, 0.0,
                                           0.0, cos,-sin,
                                           0.0, sin, cos);
   
   
}
void ossimLandSatModel::initAdjustableParameters()
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::initAdjustableParameters: entering..." << std::endl;
   resizeAdjustableParameterArray(NUM_ADJUSTABLE_PARAMS);
   int numParams = getNumberOfAdjustableParameters();
   for (int i=0; i<numParams; i++)
   {
      setAdjustableParameter(i, 0.0);
      setParameterDescription(i, PARAM_NAMES[i]);
      setParameterUnit(i,PARAM_UNITS[i]);
   }
   setParameterSigma(INTRACK_OFFSET, 500.0); //change for Landsat 5
   setParameterSigma(CRTRACK_OFFSET, 500.0); //change for Landsat 5
   setParameterSigma(LINE_GSD_CORR, 0.005);  
   setParameterSigma(SAMP_GSD_CORR, 0.005);  
   setParameterSigma(ROLL_OFFSET, 0.01);  
   setParameterSigma(YAW_OFFSET, 0.01);  
   setParameterSigma(YAW_RATE, 0.05);  
   setParameterSigma(MAP_ROTATION, 0.1);
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::initAdjustableParameters: returning..." << std::endl;
}
bool
ossimLandSatModel::setupOptimizer(const ossimString& init_file)
{
   if (traceExec())  ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::setupOptimizer(init_file): entering..." << std::endl;
   theMapProjection = 0;
   theIntrackOffset = 0.0;
   theCrtrackOffset = 0.0;
   theLineGsdCorr   = 0.0;   
   theSampGsdCorr   = 0.0;
   theRollOffset    = 0.0;
   theYawOffset     = 0.0;
   theYawRate       = 0.0;
   theMapRotation   = 0.0;
   ossimRefPtr<ossimFfL7> ff_headerp;
   if (ossimString::downcase(init_file).contains("header.dat"))
   {
      ossimRefPtr<ossimFfL5> h = new ossimFfL5(init_file); 
      ff_headerp = h.get();
      
      if (!ff_headerp->getErrorStatus())
      {
         double d = fabs(h->revb()->theUlEasting - h->revb()->theCenterEasting)/h->theGsd;
         h->theCenterImagePoint.x = static_cast<ossim_int32>(d); // d + 0.5 ???
         
         d = fabs(h->revb()->theUlNorthing - h->revb()->theCenterNorthing)/h->theGsd;
         h->theCenterImagePoint.y = static_cast<ossim_int32>(d); // d + 0.5 ???
         initFromHeader(*ff_headerp);
         
         theMapOffset.x = h->revb()->theUlEasting;
         theMapOffset.y = h->revb()->theUlNorthing;
         
      }
      else
      {
         ff_headerp = 0;
      }
   }
   else
   {
      ff_headerp=new ossimFfL7(init_file);
      if (!ff_headerp->getErrorStatus())
      {
         initFromHeader(*ff_headerp);
      }
      else
      {
         ff_headerp = 0;
      }
   }
   if(!ff_headerp.valid())
   {
      ossimFilename init_filename(init_file);
      ossimKeywordlist kwl(init_filename);
      loadState(kwl);
   }
   if (traceExec()) ossimNotify(ossimNotifyLevel_DEBUG) << "DEBUG ossimLandSatModel::setupOptimizer(init_file): Exited..." << std::endl;
   return true;
}


double ossimLandSatModel::getCenterLongitude() const
{
	return m_centerLongitude;
}

int ossimLandSatModel::getPathNumber() const
{
	return theWrsPathNumber;
}

int ossimLandSatModel::getRowNumber() const
{
	return theWrsRowNumber;
}