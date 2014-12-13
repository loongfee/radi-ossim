//*******************************************************************
//
// License:  See top level LICENSE.txt file.
//
// Author:  Long Tengfei
//
// Description:
//
// Contains definition of class ossimQVProcSupportData.
//
//*****************************************************************************
// $Id: ossimQVProcSupportData.cpp 20609 2014-01-01 12:05:13Z gpotts $

#include <radi/ossimQVProcSupportData.h>
#include <ossim/base/ossimFilename.h>
#include <ossim/base/ossimXmlDocument.h>
#include <ossim/base/ossimXmlAttribute.h>
#include <ossim/base/ossimXmlNode.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimDatum.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/base/ossimTrace.h>
#include <ossim/base/ossimNotifyContext.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <sstream>

// Define Trace flags for use within this file:
static ossimTrace traceDebug ("ossimQVProcSupportData:debug");
namespace ossimplugins
{

static const ossim_uint32  LAGRANGE_FILTER_SIZE = 8; // num samples considered
static const char* CITY_FILE = "city.txt";
static const char* DESC_FILE = "desc.xml";
static const char* EPH_FILE = "eph.txt";
static const char* GRID_FILE = "grid.txt";
static const char* HOTSPOT_FILE = "hotspot.txt";
static const char* NADIR_FILE = "nadir.txt";
static const char* PROVINCE_FILE = "province.txt";
static const char* AUX_INFO_FILE = "auxinfo";
static const char* RAW_PATH = "image\\dump\\";

ossimQVProcSupportData::ossimQVProcSupportData ()
   :
   ossimErrorStatusInterface(),
   theMetadataVersion(OSSIM_QVProc_VERSION_UNKNOWN),
   theImageID(),
   theMetadataFile(),
   theProductionDate(),
   theInstrument(),
   theInstrumentIndex(0),
   theSunAzimuth(0.0),
   theSunElevation(0.0),
   theIncidenceAngle(0.0),
   theViewingAngle(0.0),
   theSceneOrientation(0.0),
   theImageSize(0.0, 0.0),
   theRefGroundPoint(0.0, 0.0, 0.0),
   theRefImagePoint(0.0, 0.0),
   theSubImageOffset(0.0, 0.0),
   theLineSamplingPeriod(0.0),
   theReferenceTime(0.0),
   theReferenceTimeLine(0.0),
   theDetectorCount(0),
   thePixelLookAngleX(),
   thePixelLookAngleY(),
   theAttitudeSamples(),
   theAttitudeBias(),
   theAttitudeVelBias(),
   theAttitudeVelSamples(),
   theAttSampTimes(),
   theSampTimesBias(),
   thePosEcfSamples(),
   theVelEcfSamples(),
   theEphSampTimes(),
   theLineSamples(),
   theLineSampTimes(),
   theStarTrackerUsed(false),
   theSwirDataFlag(false),
   theNumBands(0),
   theAcquisitionDate(),
   theStepCount(0),
   theUlCorner(),
   theUrCorner(),
   theLrCorner(),
   theLlCorner(),
   theGeoPosImagePoints(),
   theGeoPosGroundPoints()
{
}
ossimQVProcSupportData::ossimQVProcSupportData(const ossimQVProcSupportData& rhs)
   :ossimErrorStatusInterface(rhs),
    theMetadataVersion(rhs.theMetadataVersion),
    theImageID(rhs.theImageID),
    theMetadataFile (rhs.theMetadataFile),
    theProductionDate(rhs.theProductionDate),
    theInstrument(rhs.theInstrument),
    theInstrumentIndex(rhs.theInstrumentIndex),
    theSunAzimuth(rhs.theSunAzimuth),
    theSunElevation(rhs.theSunElevation),  
    theIncidenceAngle(rhs.theIncidenceAngle),
    theViewingAngle(rhs.theViewingAngle),
    theSceneOrientation(rhs.theSceneOrientation),
    theImageSize(rhs.theImageSize),
    theRefGroundPoint(rhs.theRefGroundPoint),
    theRefImagePoint(rhs.theRefImagePoint),
    theSubImageOffset(rhs.theSubImageOffset),
    theLineSamplingPeriod(rhs.theLineSamplingPeriod),
	theReferenceTime(rhs.theReferenceTime),
	theReferenceTimeLine(rhs.theReferenceTimeLine),
    theDetectorCount(rhs.theDetectorCount),
    thePixelLookAngleX(rhs.thePixelLookAngleX),
    thePixelLookAngleY(rhs.thePixelLookAngleY),
    theAttitudeSamples(rhs.theAttitudeSamples),
	theAttitudeBias(rhs.theAttitudeBias),
	theAttitudeVelBias(rhs.theAttitudeVelBias),
	theAttitudeVelSamples(rhs.theAttitudeVelSamples),
	theAttSampTimes(rhs.theAttSampTimes),
	theSampTimesBias(rhs.theSampTimesBias),
    thePosEcfSamples(rhs.thePosEcfSamples),
    theVelEcfSamples(rhs.theVelEcfSamples),
	theEphSampTimes(rhs.theEphSampTimes),
	theLineSamples(rhs.theLineSamples),
	theLineSampTimes(rhs.theLineSampTimes),
    theStarTrackerUsed(rhs.theStarTrackerUsed),
    theSwirDataFlag (rhs.theSwirDataFlag),
    theNumBands(rhs.theNumBands),
    theAcquisitionDate(rhs.theAcquisitionDate),
    theStepCount(rhs.theStepCount),
    theUlCorner(rhs.theUlCorner),
    theUrCorner(rhs.theUrCorner),
    theLrCorner(rhs.theLrCorner),
    theLlCorner(rhs.theLlCorner),
    theGeoPosImagePoints(rhs.theGeoPosImagePoints),
    theGeoPosGroundPoints(rhs.theGeoPosGroundPoints)
{
}

ossimQVProcSupportData::ossimQVProcSupportData (const ossimFilename& descFile, bool  processSwir)
   :
   ossimErrorStatusInterface(),
   theMetadataVersion(OSSIM_QVProc_VERSION_UNKNOWN),
   theImageID(),
   theMetadataFile (descFile),
   theProductionDate(),
   theInstrument(),
   theInstrumentIndex(0),
   theSunAzimuth(0.0),
   theSunElevation(0.0),
   theIncidenceAngle(0.0),
   theViewingAngle(0.0),
   theSceneOrientation(0.0),
   theImageSize(0.0, 0.0),
   theRefGroundPoint(0.0, 0.0, 0.0),
   theRefImagePoint(0.0, 0.0),
   theSubImageOffset(0.0, 0.0),
   theLineSamplingPeriod(0.0),
   theReferenceTime(0.0),
   theReferenceTimeLine(0.0),
   theDetectorCount(0),
   thePixelLookAngleX(),
   thePixelLookAngleY(),
   theAttitudeSamples(),
   theAttitudeBias(),
   theAttitudeVelBias(),
   theAttitudeVelSamples(),
   theAttSampTimes(),
   theSampTimesBias(),
   thePosEcfSamples(),
   theVelEcfSamples(),
   theEphSampTimes(),
   theLineSamples(),
   theLineSampTimes(),
   theStarTrackerUsed(false),
   theSwirDataFlag (processSwir),
   theNumBands(0),
   theAcquisitionDate(),
   theStepCount(0),
   theUlCorner(),
   theUrCorner(),
   theLrCorner(),
   theLlCorner(),
   theGeoPosImagePoints(),
   theGeoPosGroundPoints()
{
   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "ossimQVProcSupportData::ossimQVProcSupportData: entering..."
         << std::endl;
   }

   loadXmlFile(descFile, processSwir);

   // Finished successful parse:
   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "ossimQVProcSupportData::ossimQVProcSupportData: leaving..."
         << std::endl;
   }
}

ossimQVProcSupportData::~ossimQVProcSupportData ()
{
}

ossimObject* ossimQVProcSupportData::dup()const
{
   return new ossimQVProcSupportData(*this);
}

void ossimQVProcSupportData::clearFields()
{
   clearErrorStatus();
   //theSensorID="Spot 5";
   theSensorID="";
   theMetadataVersion = OSSIM_QVProc_VERSION_UNKNOWN;
   theImageID = "";
   theMetadataFile = "";
   theProductionDate = "";
   theInstrument = "";
   theInstrumentIndex = 0;
   theSunAzimuth = 0.0;
   theSunElevation = 0.0;
   theIncidenceAngle = 0.0;
   theViewingAngle = 0.0;
   theSceneOrientation = 0.0;
   theDetectorCount = 0;
   theReferenceTime = 0.0;
   theReferenceTimeLine = 0.0;
   thePixelLookAngleX.clear();
   thePixelLookAngleY.clear();
   theAttitudeSamples.clear(); // x=pitch, y=roll, z=yaw
   theAttitudeBias.clear();
   theAttitudeVelBias.clear();
   theAttitudeVelSamples.clear();
   theAttSampTimes.clear();
   theSampTimesBias.clear();
   thePosEcfSamples.clear();
   theVelEcfSamples.clear();
   theEphSampTimes.clear();
   theLineSamples.clear();
   theLineSampTimes.clear();
   theStarTrackerUsed = false;
   theSwirDataFlag = false;
   theNumBands = 0;
   theAcquisitionDate = "";
   theStepCount = 0;

   //---
   // Corner points:
   //---

   //---
   // Geoposition Points:
   //---
   theGeoPosImagePoints.clear();
   theGeoPosGroundPoints.clear();

}

bool ossimQVProcSupportData::loadXmlFile(const ossimFilename& file,
                                            bool processSwir)
{
   static const char MODULE[] = "ossimQVProcSupportData::loadXmlFile";

   if(traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << MODULE << " DEBUG:"
         << "\nFile: " << file << std::endl;
   }
   clearFields();
   theSwirDataFlag = processSwir;
   theMetadataFile = file;

   ossim_int64 fileSize = file.fileSize();
   std::ifstream in(file.c_str(), std::ios::binary|std::ios::in);
   std::vector<char> fullBuffer;
   ossimString bufferedIo;
   if(in.good()&&(fileSize > 0))
   {
      char buf[100];
      fullBuffer.resize(fileSize);
      in.read(buf, ossim::min((ossim_int64)100, fileSize));
      if(!in.fail())
      {
         ossimString testString = ossimString(buf,
                                              buf + in.gcount());
         if(testString.contains("xml"))
         {
            in.seekg(0);
            in.read(&fullBuffer.front(), (std::streamsize)fullBuffer.size());
            if(!in.fail())
            {
               bufferedIo = ossimString(fullBuffer.begin(),
                                        fullBuffer.begin()+in.gcount());
            }
         }
      }
   }
   else
   {
      return false;
   }
   //---
   // Instantiate the XML parser:
   //---
   ossimRefPtr<ossimXmlDocument> xmlDocument;

   if(bufferedIo.empty())
   {
      xmlDocument = new ossimXmlDocument(file);
   }
   else
   {

      xmlDocument = new ossimXmlDocument;
      std::istringstream inStringStream(bufferedIo.string());
      if(!xmlDocument->read(inStringStream))
      {
         return false;
      }
   }
   if (xmlDocument->getErrorStatus())
   {
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_FATAL)
            << MODULE << " DEBUG:"
            << "ossimQVProcSupportData::loadXmlFile:"
            << "\nUnable to parse xml file" << std::endl;
      }
      setErrorStatus();
      return false;
   }

   //---
   // Check that it is a SPOT DIMAP file format
   //---

   
	ossimString xpath;
	vector<ossimRefPtr<ossimXmlNode> > xml_nodes;
   //---
   // Get the version string.  This must be performed first as it is used
   // as a key for parsing different versions.
   //---
   //if (initMetadataVersion(xmlDocument) == false)
   //{
   //   if(traceDebug())
   //   {
   //      ossimNotify(ossimNotifyLevel_FATAL)
   //         << MODULE << " DEBUG:"
   //         << "ossimQVProcSupportData::loadXmlFile:"
   //         << "\nMetadata initialization failed.  Returning false"
   //         << std::endl;
   //   }
   //   return false;
   //}

	// Get data from "PROCESSING" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/PROCESSING";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theProcessing = xml_nodes[0]->getText().toBool();

	
	// Get data from "STATION_ID" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/STATION_ID";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theStationID = xml_nodes[0]->getText();

	
	// Get data from "SPACECRAFT_ID" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/SPACECRAFT_ID";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theSpacecraftID = xml_nodes[0]->getText();
	
	// Get data from "SENSOR_ID" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/SENSOR_ID";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theSensorID = xml_nodes[0]->getText();
	theSensorID = theSpacecraftID + " " + theSensorID;
	
	// Get data from "RESAMPLE_RATE" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/RESAMPLE_RATE";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theResampleRate = xml_nodes[0]->getText().toInt();	
	
	// Get data from "START_TIME" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/START_TIME";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theStartTime = xml_nodes[0]->getText();

	// Get data from "STOP_TIME" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/STOP_TIME";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theStopTime = xml_nodes[0]->getText();

	// Get data from "COLUMNS" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/COLUMNS";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theImageSize.samp = xml_nodes[0]->getText().toInt();

	// Get data from "LINES" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/LINES";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theImageSize.line = xml_nodes[0]->getText().toInt();

	// Get data from "BANDS" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/BANDS";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theNumBands = xml_nodes[0]->getText().toInt();

	// Get data from "PYRAMID_LEVELS" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/PYRAMID_LEVELS";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	thePyramidLevels = xml_nodes[0]->getText().toInt();

	// Get data from "IMAGE_TILE_SIDE" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/IMAGE_TILE_SIDE";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theImageTileSide = xml_nodes[0]->getText().toInt();

	// Get data from "IFCITY" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/IFCITY";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theCity = xml_nodes[0]->getText().toBool();

	// Get data from "IFPROVINCE" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/IFPROVINCE";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theProvince = xml_nodes[0]->getText().toBool();

	//// Get data from "IFAUXINFO" section.
	//xml_nodes.clear();
	//xpath = "/DATASET/BASE_INFO/IFAUXINFO";
	//xmlDocument->findNodes(xpath, xml_nodes);
	//if (xml_nodes.size() == 0)
	//{
	//	setErrorStatus();
	//	if(traceDebug())
	//	{
	//		ossimNotify(ossimNotifyLevel_DEBUG)
	//		<< "DEBUG:\nCould not find: " << xpath
	//		<< endl;
	//	}
	//	return false;
	//}
	//theAuxInfo = xml_nodes[0]->getText().toBool();

	// Get data from "IFEPH" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/IFEPH";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theEph = xml_nodes[0]->getText().toBool();

	// Get data from "IFNADIR" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/IFNADIR";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theNadir = xml_nodes[0]->getText().toBool();

	// Get data from "GRID_TILE_SIDE" section.
	xml_nodes.clear();
	xpath = "/DATASET/BASE_INFO/GRID_TILE_SIDE";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
			<< "DEBUG:\nCould not find: " << xpath
			<< endl;
		}
		return false;
	}
	theGridTileSide = xml_nodes[0]->getText().toInt();


	std::vector<ossimRefPtr<ossimXmlNode> >::iterator node;
	std::vector<ossimRefPtr<ossimXmlNode> > sub_nodes;

	thePosEcfSamples.clear();
	theVelEcfSamples.clear();
	theAttSampTimes.clear();
	theEphSampTimes.clear();
	theSampTimesBias.clear();
	theAttitudeSamples.clear();
	theAttitudeBias.clear();
	theAttitudeVelBias.clear();
	theAttitudeVelSamples.clear();
	theLineSamples.clear();
	theLineSampTimes.clear();

	// Get data from "Ephemeris" section.
	xml_nodes.clear();
	xpath = "/DATASET/Ephemeris/Point";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		return false;
	}
	node = xml_nodes.begin();

	while (node != xml_nodes.end())
	{

		ossimDpt3d VP;
		sub_nodes.clear();
		xpath  = "Location/X";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VP.x = sub_nodes[0]->getText().toDouble();

		sub_nodes.clear();
		xpath  = "Location/Y";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VP.y = sub_nodes[0]->getText().toDouble();

		sub_nodes.clear();
		xpath  = "Location/Z";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VP.z = sub_nodes[0]->getText().toDouble();

		thePosEcfSamples.push_back(VP);

		ossimDpt3d VV;
		sub_nodes.clear();
		xpath = "Velocity/X";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VV.x = sub_nodes[0]->getText().toDouble();

		sub_nodes.clear();
		xpath = "Velocity/Y";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VV.y = sub_nodes[0]->getText().toDouble();

		sub_nodes.clear();
		xpath = "Velocity/Z";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VV.z = sub_nodes[0]->getText().toDouble();

		theVelEcfSamples.push_back(VV);

		sub_nodes.clear();
		xpath  = "TIME";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		theEphSampTimes.push_back(sub_nodes[0]->getText().toDouble());

		++node;
	}

	// Get data from "Attitudes" section.
	xml_nodes.clear();
	xpath = "/DATASET/Attitudes/Attitude";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		return false;
	}
	node = xml_nodes.begin();

	while (node != xml_nodes.end())
	{

		ossimDpt3d VP;
		sub_nodes.clear();
		xpath  = "Angle/PITCH";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VP.x = sub_nodes[0]->getText().toDouble();

		sub_nodes.clear();
		xpath  = "Angle/ROLL";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VP.y = sub_nodes[0]->getText().toDouble();

		sub_nodes.clear();
		xpath  = "Angle/YAW";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VP.z = sub_nodes[0]->getText().toDouble();

		theAttitudeSamples.push_back(VP);
		theAttitudeBias.push_back(ossimDpt3d(0.0,0.0,0.0));

		ossimDpt3d VV;
		sub_nodes.clear();
		xpath = "Angle_Velocity/PITCH";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VV.x = sub_nodes[0]->getText().toDouble();

		sub_nodes.clear();
		xpath = "Angle_Velocity/ROLL";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VV.y = sub_nodes[0]->getText().toDouble();

		sub_nodes.clear();
		xpath = "Angle_Velocity/YAW";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		VV.z = sub_nodes[0]->getText().toDouble();

		theAttitudeVelSamples.push_back(VV);
		theAttitudeVelBias.push_back(ossimDpt3d(0.0,0.0,0.0));

		sub_nodes.clear();
		xpath  = "TIME";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		theAttSampTimes.push_back(sub_nodes[0]->getText().toDouble());

		++node;
	}


	// Get data from "Line_Times" section.
	xml_nodes.clear();
	xpath = "/DATASET/Line_Times/Line_Time";
	xmlDocument->findNodes(xpath, xml_nodes);
	if (xml_nodes.size() == 0)
	{
		setErrorStatus();
		return false;
	}
	node = xml_nodes.begin();

	while (node != xml_nodes.end())
	{
		sub_nodes.clear();
		xpath  = "LINE";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		theLineSamples.push_back(sub_nodes[0]->getText().toDouble());

		sub_nodes.clear();
		xpath  = "TIME";
		(*node)->findChildNodes(xpath, sub_nodes);
		if (sub_nodes.size() == 0)
		{
			setErrorStatus();
			return false;
		}
		theLineSampTimes.push_back(sub_nodes[0]->getText().toDouble());

		++node;
	}
	
	if(theCity)
	{
		ossimFilename cityFile = theMetadataFile.path()+"\\"+CITY_FILE;
		if(cityFile.exists())
		{
			readCities(cityFile);
		}else{
			setErrorStatus();
			if(traceDebug())
			{
				ossimNotify(ossimNotifyLevel_DEBUG)
				<< "DEBUG:\nCould not find: " << cityFile
				<< endl;
			}			
		}
	}
	
	if(theProvince)
	{
		ossimFilename provinceFile = theMetadataFile.path()+"\\" + PROVINCE_FILE;
		if(provinceFile.exists())
		{
			readProvinces(provinceFile);
		}else{
			setErrorStatus();
			if(traceDebug())
			{
				ossimNotify(ossimNotifyLevel_DEBUG)
				<< "DEBUG:\nCould not find: " << provinceFile
				<< endl;
			}			
		}
	}
	
	//if(theAuxInfo)
	//{
	//	ossimFilename auxInfoFile = theMetadataFile.path()+"\\" + AUX_INFO_FILE;
	//	if(auxInfoFile.exists())
	//	{
	//		
	//	}else{
	//		setErrorStatus();
	//		if(traceDebug())
	//		{
	//			ossimNotify(ossimNotifyLevel_DEBUG)
	//			<< "DEBUG:\nCould not find: " << auxInfoFile
	//			<< endl;
	//		}			
	//	}
	//}
	
	//if(theEph)
	//{
	//	ossimFilename ephFile = theMetadataFile.path()+"\\" + EPH_FILE;
	//	if(ephFile.exists())
	//	{
	//		readEph(ephFile);
	//	}else{
	//		setErrorStatus();
	//		if(traceDebug())
	//		{
	//			ossimNotify(ossimNotifyLevel_DEBUG)
	//			<< "DEBUG:\nCould not find: " << ephFile
	//			<< endl;
	//		}			
	//	}
	//}
	
	if(theNadir)
	{
		ossimFilename nadirFile = theMetadataFile.path()+"\\" + NADIR_FILE;
		if(nadirFile.exists())
		{
			readNadir(nadirFile);
		}else{
			setErrorStatus();
			if(traceDebug())
			{
				ossimNotify(ossimNotifyLevel_DEBUG)
				<< "DEBUG:\nCould not find: " << nadirFile
				<< endl;
			}			
		}
	}

   if (traceDebug())
   {
      printInfo(ossimNotify(ossimNotifyLevel_DEBUG));

      ossimNotify(ossimNotifyLevel_DEBUG)
         << MODULE << " DEBUG: exited..."
         << std::endl;
   }

   theUlCorner = ossimGpt(0.0, 0.0);
   theUrCorner = ossimGpt(0.0, theImageSize.samp);
   theLrCorner = ossimGpt(theImageSize.line, theImageSize.samp);
   theLlCorner = ossimGpt(theImageSize.line, 0.0);

   return true;
}

bool ossimQVProcSupportData::readCities(ossimFilename cityFile)
{
	theCityList.clear();
	ifstream os(cityFile.c_str());
	char strtmp[2048];
	ossimString str;
	int iLine = 0;
	while(os.getline(strtmp,2048))
	{
		str=strtmp;
		if (0 == iLine++)
		{
			if(0 != strcmp(str.upcase().c_str(), "POINT"))
			{
				setErrorStatus();
				if(traceDebug())
				{
					ossimNotify(ossimNotifyLevel_DEBUG)
					<< "DEBUG:\n"
					<<CITY_FILE
					<<" is not of POINT features"
					<< endl;
				}
				os.close();
				return false;
			}
			continue;
		}
		vector<ossimString> strList = str.split(" ", true);
		if (strList.size() != 4)
		{
			setErrorStatus();
			if(traceDebug())
			{
				ossimNotify(ossimNotifyLevel_DEBUG)
				<< "DEBUG:\n"
				<<CITY_FILE
				<<" is not of correct format"
				<< endl;
			}
			os.close();
			return false;
		}
		cityStruct cs;
		cs.imageX = strList[0].toDouble();
		cs.imageY = strList[1].toDouble();
		cs.strName = strList[2];
		cs.level = strList[3].toInt();
		theCityList.push_back(cs);
	}
	os.close();
	return true;
}

bool ossimQVProcSupportData::readProvinces(ossimFilename provinceFile)
{
	theProvinceList.clear();
	ifstream os(provinceFile.c_str());
	char strtmp[2048];
	ossimString str;
	int iLine = 0;
	while(os.getline(strtmp,2048))
	{
		str=strtmp;
		if (0 == iLine++)
		{
			if(0 != strcmp(str.upcase().c_str(), "POLYLINE"))
			{
				setErrorStatus();
				if(traceDebug())
				{
					ossimNotify(ossimNotifyLevel_DEBUG)
					<< "DEBUG:\n"
					<<PROVINCE_FILE
					<<" is not of POLYLINE features"
					<< endl;
				}
				os.close();
				return false;
			}
			continue;
		}
		vector<ossimString> strList = str.split(" ", true);
		std::vector<ossimDpt> ptList;
		for (int i = 0; i < (int)strList.size(); i+=2)
		{
			ptList.push_back(ossimDpt(strList[i].toDouble(), strList[i+1].toDouble()));
		}
		theProvinceList.push_back(ptList);
	}
	os.close();
	return true;
}


//bool ossimQVProcSupportData::readEph(ossimFilename ephFile)
//{	
//	thePosEcfSamples.clear();
//	theVelEcfSamples.clear();
//	theEphSampTimes.clear();
//	ifstream os(ephFile.c_str());
//	char strtmp[2048];
//	ossimString str;
//	int iLine = 0;
//	while(os.getline(strtmp,2048))
//	{
//		str=strtmp;
//		ossimString separatorList = "[ ]";
//		vector<ossimString> strList = str.split(separatorList, true);
//		if (strList.size() != 7)
//		{
//			setErrorStatus();
//			if(traceDebug())
//			{
//				ossimNotify(ossimNotifyLevel_DEBUG)
//				<< "DEBUG:\n"
//				<< EPH_FILE
//				<<" is not of correct format"
//				<< endl;
//			}
//			os.close();
//			return false;
//		}
//
//		// time
//		theEphSampTimes.push_back(strList[0].toDouble());
//
//		// [x y z]
//		//vector<ossimString> strSubList = strList[1].split(" ", true);		
//		//if (strSubList.size() != 3)
//		//{
//		//	setErrorStatus();
//		//	if(traceDebug())
//		//	{
//		//		ossimNotify(ossimNotifyLevel_DEBUG)
//		//		<< "DEBUG:\n"
//		//		<< EPH_FILE
//		//		<<" is not of correct format"
//		//		<< endl;
//		//	}
//		//	os.close();
//		//	return false;
//		//}
//		thePosEcfSamples.push_back(ossimDpt3d(strList[1].toDouble(), strList[2].toDouble(), strList[3].toDouble()));
//
//		// [dx dy dz]
//		//strSubList = strList[2].split(" ", true);		
//		//if (strSubList.size() != 3)
//		//{
//		//	setErrorStatus();
//		//	if(traceDebug())
//		//	{
//		//		ossimNotify(ossimNotifyLevel_DEBUG)
//		//		<< "DEBUG:\n"
//		//		<< EPH_FILE
//		//		<<" is not of correct format"
//		//		<< endl;
//		//	}
//		//	os.close();
//		//	return false;
//		//}
//		theVelEcfSamples.push_back(ossimDpt3d(strList[4].toDouble(), strList[5].toDouble(), strList[6].toDouble()));
//
//		iLine++;
//	}
//	os.close();
//	return true;
//}

bool ossimQVProcSupportData::readEph(ossimFilename ephFile)
{	
	thePosEcfSamples.clear();
	theVelEcfSamples.clear();
	theAttSampTimes.clear();
	theSampTimesBias.clear();
	theAttitudeSamples.clear();
	theAttitudeBias.clear();
	theAttitudeVelBias.clear();
	theAttitudeVelSamples.clear();
	theEphSampTimes.clear();
	theLineSamples.clear();
	theLineSampTimes.clear();
	ifstream os(ephFile.c_str());
	char strtmp[2048];
	ossimString str;
	int iLine = 0;
	while(os.getline(strtmp,2048))
	{
		str=strtmp;
		ossimString separatorList = " ";
		vector<ossimString> strList = str.split(separatorList, true);
		if (strList.size() < 14)
		{
			setErrorStatus();
			if(traceDebug())
			{
				ossimNotify(ossimNotifyLevel_DEBUG)
					<< "DEBUG:\n"
					<< EPH_FILE
					<<" is not of correct format"
					<< endl;
			}
			os.close();
			return false;
		}

		// time
		//theEphSampTimes.push_back(strList[10].toDouble());
		theAttSampTimes.push_back(strList[0].toDouble());

		thePosEcfSamples.push_back(ossimDpt3d(strList[1].toDouble(), strList[2].toDouble(), strList[3].toDouble()));

		theVelEcfSamples.push_back(ossimDpt3d(strList[4].toDouble(), strList[5].toDouble(), strList[6].toDouble()));

		// roll-pitch-yaw --> pitch-roll-yaw
		theAttitudeSamples.push_back(ossimDpt3d(strList[8].toDouble(), strList[7].toDouble(), strList[9].toDouble()));
		theAttitudeVelSamples.push_back(ossimDpt3d(strList[11].toDouble(), strList[10].toDouble(), strList[12].toDouble()));
		//theAttitudeSamples.push_back(ossimDpt3d(strList[7].toDouble(), strList[8].toDouble(), strList[9].toDouble()));
		//theAttitudeVelSamples.push_back(ossimDpt3d(strList[10].toDouble(), strList[11].toDouble(), strList[12].toDouble()));

		theReferenceTime = strList[13].toDouble();
		theReferenceTimeLine = strList[0].toDouble();

		theAttitudeBias.push_back((ossimDpt3d(0.0, 0.0, 0.0)));
		theAttitudeVelBias.push_back(ossimDpt3d(0.0,0.0,0.0));
		theSampTimesBias.push_back(0.0);

		iLine++;
	}
	updatePosEcfCoeff();
	updateVelEcfCoeff();
	os.close();
	return true;
}

bool ossimQVProcSupportData::readNadir(ossimFilename nadirFile)
{	
	theEphSampLines.clear();
	theNadirList.clear();
	ifstream os(nadirFile.c_str());
	char strtmp[2048];
	ossimString str;
	int iLine = 0;
	while(os.getline(strtmp,2048))
	{
		str=strtmp;
		ossimString separatorList = " ";
		vector<ossimString> strList = str.split(separatorList, true);
		if (strList.size() != 3)
		{
			setErrorStatus();
			if(traceDebug())
			{
				ossimNotify(ossimNotifyLevel_DEBUG)
				<< "DEBUG:\n"
				<< NADIR_FILE
				<<" is not of correct format"
				<< endl;
			}
			os.close();
			return false;
		}

		// lines
		theEphSampLines.push_back(strList[0].toDouble());

		// longitude lattitude
		theNadirList.push_back(ossimDpt(strList[1].toDouble(), strList[2].toDouble()));

		iLine++;
	}
	os.close();
	return true;
}


void ossimQVProcSupportData::updatePosEcfCoeff()
{
	int observation_num = (int)theAttSampTimes.size();
	int coeff_num = 12;
	arma::mat A = arma::zeros(observation_num*3, coeff_num);
	arma::vec L(observation_num*3);
	for (int i = 0;i < observation_num;++i)
	{
		double t0 = 1.0;
		double t1 = theAttSampTimes[i];
		double t2 = t1 * t1;
		double t3 = t1 * t1 * t1;
		A(i*3, 0) = t0;
		A(i*3, 1) = t1;
		A(i*3, 2) = t2;
		A(i*3, 3) = t3;
		A(i*3+1, 4) = t0;
		A(i*3+1, 5) = t1;
		A(i*3+1, 6) = t2;
		A(i*3+1, 7) = t3;
		A(i*3+2, 8) = t0;
		A(i*3+2, 9) = t1;
		A(i*3+2, 10) = t2;
		A(i*3+2, 11) = t3;
		L(i*3) = thePosEcfSamples[i].x;
		L(i*3+1) = thePosEcfSamples[i].y;
		L(i*3+2) = thePosEcfSamples[i].z;
	}
	thePosEcfCoeff = arma::solve(A, L);
}

void ossimQVProcSupportData::updateVelEcfCoeff()
{
	int observation_num = (int)theAttSampTimes.size();
	int coeff_num = 12;
	arma::mat A = arma::zeros(observation_num*3, coeff_num);
	arma::vec L(observation_num*3);
	for (int i = 0;i < observation_num;++i)
	{
		double t0 = 1.0;
		double t1 = theAttSampTimes[i];
		double t2 = t1 * t1;
		double t3 = t1 * t1 * t1;
		A(i*3, 0) = t0;
		A(i*3, 1) = t1;
		A(i*3, 2) = t2;
		A(i*3, 3) = t3;
		A(i*3+1, 4) = t0;
		A(i*3+1, 5) = t1;
		A(i*3+1, 6) = t2;
		A(i*3+1, 7) = t3;
		A(i*3+2, 8) = t0;
		A(i*3+2, 9) = t1;
		A(i*3+2, 10) = t2;
		A(i*3+2, 11) = t3;
		L(i*3) = theVelEcfSamples[i].x;
		L(i*3+1) = theVelEcfSamples[i].y;
		L(i*3+2) = theVelEcfSamples[i].z;
	}
	theVelEcfCoeff = arma::solve(A, L);
}


void ossimQVProcSupportData::getLineTime(const ossim_float64& line,
                                            ossim_float64& time)  const
{
   if (theLineSampTimes.empty())
   {
     time = 0.0;
     return;
   }

   if ((line <  theLineSamples.front()) ||
       (line >= theLineSamples.back() ))
   {
      extrapolateTime(line, time);
      return;
   }

   //***
   // Search the attitude sampling time array for surrounding samples:
   //***
   int i=0;
   while ((i < (int)theLineSamples.size()) &&
          (theLineSamples[i] < line)) ++i;
   --i;

   //***
   // Linearly interpolate attitudes angles:
   //***
   ossim_float64 dLine1   = line - theLineSamples[i];
   ossim_float64 dLine0   = theLineSamples[i+1] - line;
   ossim_float64 dLine    = theLineSamples[i+1] - theLineSamples[i];

   time = (theLineSampTimes[i+1]*dLine1 + theLineSampTimes[i]*dLine0)/dLine;
}


void ossimQVProcSupportData::extrapolateTime(const ossim_float64& line, ossim_float64& time) const
{
	time = 0.0;
	int last_samp = (int) theLineSamples.size() - 1;
	if (last_samp < 1)
		return;
	
	// Determine whether extrapolating at the front or the back of the range:
	if (line < theLineSamples.front())
	{
		double dLine = theLineSamples[1] - theLineSamples[0];
		double dTime = theLineSampTimes[1] - theLineSampTimes[0];
		double dTime_dLine = dTime/dLine;
		double delta_line = line - theLineSamples[0];
		time = theLineSampTimes[0] + (dTime_dLine*delta_line);
	}
	else if (line >= theLineSamples.back())
	{

		double dLine = theLineSamples[last_samp] - theLineSamples[last_samp-1];
		double dTime = theLineSampTimes[last_samp] - theLineSampTimes[last_samp-1];
		double dTime_dLine = dTime/dLine;
		double delta_line = line - theLineSamples[last_samp];
		time = theLineSampTimes[last_samp] + (dTime_dLine*delta_line);
	}

	return;
}

void ossimQVProcSupportData::getPositionEcf(ossim_uint32 sample,
                                               ossimEcefPoint& pe)  const
{
   pe.makeNan();

   if (thePosEcfSamples.size() < theDetectorCount)
   {
      if(theImageSize.samp > 0)
      {
         double t = 0.0;
         double tempIdx = 0.0;
         double tempIdxFraction = 0.0;
         t = static_cast<double>(sample)/
            static_cast<double>(theDetectorCount-1);
         tempIdx = (thePosEcfSamples.size()-1)*t;
         tempIdxFraction = tempIdx - (ossim_int32)tempIdx;
         ossim_uint32 idxStart = (ossim_uint32)tempIdx;
         ossim_uint32 idxEnd = (ossim_uint32)ceil(tempIdx);
         if(idxEnd >= thePosEcfSamples.size())
         {
            idxEnd = (ossim_uint32)thePosEcfSamples.size()-1;
         }
         if(idxStart > idxEnd)
         {
            idxStart = idxEnd;
         }
         pe = ossimEcefPoint(thePosEcfSamples[idxStart].x +tempIdxFraction*( thePosEcfSamples[idxEnd].x - thePosEcfSamples[idxStart].x),
                             thePosEcfSamples[idxStart].y +tempIdxFraction*( thePosEcfSamples[idxEnd].y - thePosEcfSamples[idxStart].y),
                             thePosEcfSamples[idxStart].z +tempIdxFraction*( thePosEcfSamples[idxEnd].z - thePosEcfSamples[idxStart].z));

      }
   }
   else if(thePosEcfSamples.size() == theDetectorCount)
   {
      pe = ossimEcefPoint(thePosEcfSamples[sample].x,
                          thePosEcfSamples[sample].y,
                          thePosEcfSamples[sample].z);
   }
}

void ossimQVProcSupportData::getPositionEcf(const ossim_float64& time,
                                               ossimEcefPoint& pe)  const
{
	//double t0 = 1.0;
	//double t1 = time;
	//double t2 = t1 * t1;
	//double t3 = t1 * t1 * t1;
	//double x = t0*thePosEcfCoeff(0) 
	//	+ t1*thePosEcfCoeff(1)
	//	+ t2*thePosEcfCoeff(2)
	//	+ t3*thePosEcfCoeff(3);
	//double y = t0*thePosEcfCoeff(4) 
	//	+ t1*thePosEcfCoeff(5)
	//	+ t2*thePosEcfCoeff(6)
	//	+ t3*thePosEcfCoeff(7);
	//double z = t0*thePosEcfCoeff(8) 
	//	+ t1*thePosEcfCoeff(9)
	//	+ t2*thePosEcfCoeff(10)
	//	+ t3*thePosEcfCoeff(11);
	//pe = ossimEcefPoint(x,y,z);
	//return;
	
	if (theEphSampTimes.empty())
   {
     pe.makeNan();
     return;
   }

//	ossim_float64 line_time = 4.369e-3;
   if ((time <  theEphSampTimes.front()) ||
       (time >= theEphSampTimes.back()))
   {
	   ossimDpt3d tempPt;
	   int last_samp = (int) theEphSampTimes.size() - 1;
	   if (last_samp < 1)
		   return;
	   
	   // Determine whether extrapolating at the front or the back of the range:
	   if (time < theEphSampTimes.front())
	   {
		   double dt = (time - theEphSampTimes.front());
		   tempPt = thePosEcfSamples[0] + (theVelEcfSamples[0]*dt);
	   }
	   else if (time >= theEphSampTimes.back())
	   {
		   double dt = (time - theEphSampTimes.back());
		   tempPt = thePosEcfSamples[last_samp] + (theVelEcfSamples[last_samp]*dt);
	   }

	   pe = ossimEcefPoint(tempPt.x,
		   tempPt.y,
		   tempPt.z);
	   return;
   }

   //***
   // Search the attitude sampling time array for surrounding samples:
   //***
   int i=0;
   while ((i < (int)theEphSampTimes.size()) &&
          (theEphSampTimes[i] < time)) ++i;
   --i;
   ossim_float64 dt1   = time - theEphSampTimes[i];
   ossim_float64 dt0   = theEphSampTimes[i+1] - time;
   ossim_float64 dt    = theEphSampTimes[i+1] - theEphSampTimes[i];
   ossim_float64 dtt1 = dt1;
   ossim_float64 dtt0 = dt0;
   ossim_float64 dtt = dt;

   //at = (theAttitudeSamples[i+1]*dt1 + theAttitudeSamples[i]*dt0)/dt;

   ossimDpt3d tempPt = thePosEcfSamples[i]*(1.0+2.0*dtt1/dtt)*dtt0*dtt0/(dtt*dtt)
	   + thePosEcfSamples[i+1]*(1.0+2.0*dtt0/dtt)*dtt1*dtt1/(dtt*dtt)
	   + theVelEcfSamples[i]*dtt1*dtt0*dtt0/(dtt*dtt)
	   - theVelEcfSamples[i+1]*dtt0*dtt1*dtt1/(dtt*dtt);

   //ossimDpt3d tempPt;

   //if((thePosEcfSamples.size() < 8)||
   //   (theEphSampTimes.size() < 8))
   //{
   //   getBilinearInterpolation(time, thePosEcfSamples, theEphSampTimes, tempPt);
   //}
   //else
   //{
   //   getLagrangeInterpolation(time, thePosEcfSamples, theEphSampTimes, tempPt);
   //}

   pe = ossimEcefPoint(tempPt.x,
                       tempPt.y,
                       tempPt.z);
}

void ossimQVProcSupportData::getVelocityEcf(ossim_uint32 sample, ossimEcefPoint& ve)  const
{
   ve.makeNan();

   if (theVelEcfSamples.size() < theDetectorCount)
   {
      if(theImageSize.samp > 0)
      {
         double t = 0.0;
         double tempIdx = 0.0;
         double tempIdxFraction = 0.0;
         t = static_cast<double>(sample)/
            static_cast<double>(theDetectorCount-1);
         tempIdx = (theVelEcfSamples.size()-1)*t;
         tempIdxFraction = tempIdx - (ossim_int32)tempIdx;
          ossim_uint32 idxStart = (ossim_uint32)tempIdx;
         ossim_uint32 idxEnd = (ossim_uint32)ceil(tempIdx);
         if(idxEnd >= theVelEcfSamples.size())
         {
            idxEnd = (ossim_uint32)theVelEcfSamples.size()-1;
         }
         if(idxStart > idxEnd)
         {
            idxStart = idxEnd;
         }
         ve = ossimEcefPoint(theVelEcfSamples[idxStart].x +tempIdxFraction*( theVelEcfSamples[idxEnd].x - theVelEcfSamples[idxStart].x),
                             theVelEcfSamples[idxStart].y +tempIdxFraction*( theVelEcfSamples[idxEnd].y - theVelEcfSamples[idxStart].y),
                             theVelEcfSamples[idxStart].z +tempIdxFraction*( theVelEcfSamples[idxEnd].z - theVelEcfSamples[idxStart].z));

      }

   }
   else if(theVelEcfSamples.size() == theDetectorCount)
   {
      ve = ossimEcefPoint(theVelEcfSamples[sample].x,
                          theVelEcfSamples[sample].y,
                          theVelEcfSamples[sample].z);
   }
}

void ossimQVProcSupportData::getVelocityEcf(const ossim_float64& time,
                                               ossimEcefPoint& ve)  const
{
	//double t0 = 1.0;
	//double t1 = time;
	//double t2 = t1 * t1;
	//double t3 = t1 * t1 * t1;
	//double x = t0*theVelEcfCoeff(0) 
	//	+ t1*theVelEcfCoeff(1)
	//	+ t2*theVelEcfCoeff(2)
	//	+ t3*theVelEcfCoeff(3);
	//double y = t0*theVelEcfCoeff(4) 
	//	+ t1*theVelEcfCoeff(5)
	//	+ t2*theVelEcfCoeff(6)
	//	+ t3*theVelEcfCoeff(7);
	//double z = t0*theVelEcfCoeff(8) 
	//	+ t1*theVelEcfCoeff(9)
	//	+ t2*theVelEcfCoeff(10)
	//	+ t3*theVelEcfCoeff(11);
	//ve = ossimEcefPoint(x,y,z);
	//return;

   //ossimDpt3d tempPt;

   //if((theVelEcfSamples.size() < 8) ||
   //   (theEphSampTimes.size() < 8))
   //{
   //   getBilinearInterpolation (time, theVelEcfSamples, theEphSampTimes, tempPt);
   //}
   //else
   //{
   //   getLagrangeInterpolation (time, theVelEcfSamples, theEphSampTimes, tempPt);
   //}

   //ve = ossimEcefPoint(tempPt.x,
   //                    tempPt.y,
   //                    tempPt.z);
	
	if (theEphSampTimes.empty())
   {
     ve.makeNan();
     return;
   }
	ossimDpt3d tempPt;
	//ossim_float64 line_time = 4.369e-3;
   if ((time <  theEphSampTimes.front()) ||
       (time >= theEphSampTimes.back()))
   {
	   ossimDpt3d tempPt;
	   int last_samp = (int) theEphSampTimes.size() - 1;
	   if (last_samp < 1)
		   return;
	   
	   // Determine whether extrapolating at the front or the back of the range:
	   if (time < theEphSampTimes.front())
	   {
		    double dt = theEphSampTimes[1] - theEphSampTimes[0];
		    ossimDpt3d dAtt = theVelEcfSamples[1] - theVelEcfSamples[0];
		    ossimDpt3d dAtt_dt = dAtt/dt;
		    double delta_t = time - theEphSampTimes[0];
		    tempPt = theVelEcfSamples[0] + (dAtt_dt*delta_t);
	   }
	   else if (time >= theEphSampTimes.back())
	   {
		   double dt = theEphSampTimes[last_samp] - theEphSampTimes[last_samp-1];
		   ossimDpt3d dAtt = theVelEcfSamples[last_samp] - theVelEcfSamples[last_samp-1];
		   ossimDpt3d dAtt_dt = dAtt/dt;
		   double delta_t = time - theEphSampTimes[last_samp];
			tempPt = theVelEcfSamples[last_samp] + (dAtt_dt*delta_t);
	   }

	   ve = ossimEcefPoint(tempPt.x,
		   tempPt.y,
		   tempPt.z);
	   return;
   }

   //***
   // Search the attitude sampling time array for surrounding samples:
   //***
   int i=0;
   while ((i < (int)theEphSampTimes.size()) &&
          (theEphSampTimes[i] < time)) ++i;
   --i;
   ossim_float64 dt1   = time - theEphSampTimes[i];
   ossim_float64 dt0   = theEphSampTimes[i+1] - time;
   ossim_float64 dt    = theEphSampTimes[i+1] - theEphSampTimes[i];
   ossim_float64 dtt1 = dt1;
   ossim_float64 dtt0 = dt0;
   ossim_float64 dtt = dt;

   tempPt = (theVelEcfSamples[i+1]*dt1 + theVelEcfSamples[i]*dt0)/dt;


   ve = ossimEcefPoint(tempPt.x,
                       tempPt.y,
                       tempPt.z);
}

void ossimQVProcSupportData::getAttitude(const ossim_float64& time,
                                            ossimDpt3d& at)  const
{
	double delta_t = 0.0;
	if (theSensorID.upcase().contains("HJ-1A"))
	{
		//2008-9-6 12:14:38.15 (bj)
		//2008-9-6 4:14:38.15 (utc)
		delta_t = ((1710*24.0+4)*60.0+14)*60.0+38.15;
	}
	else if(theSensorID.upcase().contains("HJ-1B"))
	{
		//2008-9-6 12:16:15.79 (bj)
		//2008-9-6 4:16:15.79 (utc)
		delta_t = ((1710*24.0+4)*60.0+16)*60.0+15.79;
	}
	double t = time - delta_t;
   if (theAttSampTimes.empty())
   {
     at.makeNan();
     return;
   }

   if ((t <  getAttSampTime(0)) ||
       (t >= getAttSampTime(theAttSampTimes.size()-1)))
   {
      extrapolateAttitude(t, at);
      return;
   }

   //***
   // Search the attitude sampling time array for surrounding samples:
   //***
   int i=0;
   while ((i < (int)theAttSampTimes.size()) &&
          getAttSampTime(i) < t) ++i;
   --i;

   //***
   // Linearly interpolate attitudes angles:
   //***
   //ossim_float64 line_time = 4.369e-3;
   ossim_float64 dt1   = t - getAttSampTime(i);
   ossim_float64 dt0   = getAttSampTime(i+1) - t;
   ossim_float64 dt    = getAttSampTime(i+1) - getAttSampTime(i);
   ossim_float64 dtt1 = dt1;
   ossim_float64 dtt0 = dt0;
   ossim_float64 dtt = dt;

   //at = (theAttitudeSamples[i+1]*dt1 + theAttitudeSamples[i]*dt0)/dt;

   at = getAttitude(i)*(1.0+2.0*dtt1/dtt)*dtt0*dtt0/(dtt*dtt)
	   + getAttitude(i+1)*(1.0+2.0*dtt0/dtt)*dtt1*dtt1/(dtt*dtt)
	   + getAttitudeVel(i)*dtt1*dtt0*dtt0/(dtt*dtt)
	   - getAttitudeVel(i+1)*dtt0*dtt1*dtt1/(dtt*dtt);
}

void ossimQVProcSupportData::extrapolateAttitude(const ossim_float64& time, ossimDpt3d& at) const
{
   at.makeNan();
   int last_samp = (int) theAttSampTimes.size() - 1;
   if (last_samp < 1)
      return;

   //ossimDpt3d dAtt, dAtt_dt;
   //double dt, delta_t;

   //ossim_float64 line_time = 4.369e-3;
   // Determine whether extrapolating at the front or the back of the range:
   if (time < getAttSampTime(0))
   {
   //   dt = theAttSampTimes[1] - theAttSampTimes[0];
   //   dAtt = (theAttitudeSamples[1]+theAttitudeBias[1]) - (theAttitudeSamples[0]+theAttitudeBias[0]);
   //   dAtt_dt = dAtt/dt;
   //   delta_t = time - theAttSampTimes[0];
   //   //at = (theAttitudeSamples[0]+theAttitudeBias[0]) + (dAtt_dt*delta_t);

	  //int i = 0;
	  //ossim_float64 dt1   = time - theAttSampTimes[i];
	  //ossim_float64 dt0   = theAttSampTimes[i+1] - time;
	  //ossim_float64 dt    = theAttSampTimes[i+1] - theAttSampTimes[i];
	  //ossim_float64 dtt1 = dt1 * line_time;
	  //ossim_float64 dtt0 = dt0 * line_time;
	  //ossim_float64 dtt = dt * line_time;

	  //at = (theAttitudeSamples[i+1]*dt1 + theAttitudeSamples[i]*dt0)/dt;

	  ////at = (theAttitudeSamples[i]+theAttitudeBias[i])*(1.0+2.0*dtt1/dtt)*dtt0*dtt0/(dtt*dtt)
		 //// + (theAttitudeSamples[i+1]+theAttitudeBias[i+1])*(1.0+2.0*dtt0/dtt)*dtt1*dtt1/(dtt*dtt)
		 //// + theAttitudeVelSamples[i]*dtt1*dtt0*dtt0/(dtt*dtt)
		 //// - theAttitudeVelSamples[i+1]*dtt0*dtt1*dtt1/(dtt*dtt);
	   double dt = (time - getAttSampTime(0));
	   at = getAttitude(0) + getAttitudeVel(0)*dt;
   }
   else if (time >= getAttSampTime(theAttSampTimes.size()-1))
   {
   //   dt = theAttSampTimes[last_samp] - theAttSampTimes[last_samp-1];
   //   dAtt = (theAttitudeSamples[last_samp]+theAttitudeBias[last_samp]) - (theAttitudeSamples[last_samp-1]+theAttitudeBias[last_samp-1]);
   //   dAtt_dt = dAtt/dt;
   //   delta_t = time - theAttSampTimes[last_samp];
	  ////at = (theAttitudeSamples[last_samp]+theAttitudeBias[last_samp]) + (dAtt_dt*delta_t);

	  //int i = last_samp-1;
	  //ossim_float64 line_time = 4.369e-3;
	  //ossim_float64 dt1   = time - theAttSampTimes[i];
	  //ossim_float64 dt0   = theAttSampTimes[i+1] - time;
	  //ossim_float64 dt    = theAttSampTimes[i+1] - theAttSampTimes[i];
	  //ossim_float64 dtt1 = dt1 * line_time;
	  //ossim_float64 dtt0 = dt0 * line_time;
	  //ossim_float64 dtt = dt * line_time;

	  //at = (theAttitudeSamples[i+1]*dt1 + theAttitudeSamples[i]*dt0)/dt;

	  ////at = (theAttitudeSamples[i]+theAttitudeBias[i])*(1.0+2.0*dtt1/dtt)*dtt0*dtt0/(dtt*dtt)
		 //// + (theAttitudeSamples[i+1]+theAttitudeBias[i+1])*(1.0+2.0*dtt0/dtt)*dtt1*dtt1/(dtt*dtt)
		 //// + theAttitudeVelSamples[i]*dtt1*dtt0*dtt0/(dtt*dtt)
	  //// - theAttitudeVelSamples[i+1]*dtt0*dtt1*dtt1/(dtt*dtt);
	  ossim_float64 dt = (time - getAttSampTime(last_samp));
	  at = getAttitude(last_samp) + getAttitudeVel(last_samp)*dt;
   }

   return;
}

void ossimQVProcSupportData::getPixelLookAngleX(ossim_uint32 sample,
                                                   ossim_float64& pa) const
{
   if (sample >= thePixelLookAngleX.size())
   {
      setErrorStatus();
      pa = ossim::nan();

      return;
   }

   pa = thePixelLookAngleX[sample];
}

void ossimQVProcSupportData::getPixelLookAngleX(const ossim_float64& sample,
                                                   ossim_float64& pa) const
{
   ossim_uint32 s = static_cast<ossim_uint32>(sample);
   getInterpolatedLookAngle(s, thePixelLookAngleX, pa);
}

void ossimQVProcSupportData::getPixelLookAngleY(ossim_uint32 sample,
                                                   ossim_float64& pa) const
{
   if (sample >= thePixelLookAngleY.size())
   {
      setErrorStatus();
      pa = ossim::nan();
      return;
   }

   pa = thePixelLookAngleY[sample];
}

void ossimQVProcSupportData::getPixelLookAngleY(const ossim_float64& sample,
                                                   ossim_float64& pa) const
{
   ossim_uint32 s = static_cast<ossim_uint32>(sample);
   getInterpolatedLookAngle(s, thePixelLookAngleY, pa);
}

void ossimQVProcSupportData::getInterpolatedLookAngle(
   const ossim_float64& p,
   const std::vector<ossim_float64>& angles,
   ossim_float64& la) const
{
   if ((p < 0.0) || (p >= (ossim_float64) angles.size()))
   {
      setErrorStatus();
      la = ossim::nan();
      return;
   }

   ossim_float64 p0 = floor(p);
   ossim_float64 p1 = ceil (p);

   if (p0 == p1)
   {
      la = angles[(int) p0];
   }
   else
   {
      ossim_float64 angle_0 = angles[(int) p0];
      ossim_float64 angle_1 = angles[(int) p1];

      la = (angle_0*(p1-p) + angle_1*(p-p0))/(p1-p0);
   }
}

void ossimQVProcSupportData::getBilinearInterpolation(
   const ossim_float64& time,
   const std::vector<ossimDpt3d>& V,
   const std::vector<ossim_float64>& T,
   ossimDpt3d& li) const
{
   ossim_uint32 samp0 = 0;
   while ((samp0 < T.size()) && (T[samp0] < time)) ++samp0;

   if(samp0==0)
   {
      li = V[0];
   }
   else if(samp0 == T.size())
   {
      li = V[T.size()-1];
   }
   else
   {
      double t = (T[samp0-1]-time)/(T[samp0-1] - T[samp0]);

      li = V[samp0-1] + (V[samp0]-V[samp0-1])*t;
   }
}

void ossimQVProcSupportData::getLagrangeInterpolation(
   const ossim_float64& time,
   const std::vector<ossimDpt3d>& V,
   const std::vector<ossim_float64>& T,
   ossimDpt3d& li) const

{
//    std::cout << "V size = " << V.size() << std::endl
//              << "T size = " << T.size() << std::endl;

   ossim_uint32 filter_size = 8;
   //
   // Verify that t is within allowable range:
   //
   ossim_uint32 lagrange_half_filter = 4;

   if(T.size() <= filter_size)
   {
      filter_size = (ossim_uint32)T.size()/2;
      lagrange_half_filter = filter_size/2;
   }
   if ((time <  T[lagrange_half_filter]) ||
       (time >= T[T.size()-lagrange_half_filter] ))
   {
      setErrorStatus();
      li.makeNan();

      return;
   }

   //***
   // Search the sampling time array for surrounding samples:
   //***
   ossim_uint32 samp0 = lagrange_half_filter;
   while ((samp0 < T.size()) && (T[samp0] < time)) ++samp0;

   //***
   // Do not use sample if it falls in neighborhood of desired time:
   //***
   ossim_uint32 bump = 0;
   if (fabs(T[samp0] - time) < theLineSamplingPeriod/2.0)
      bump = 1;

   samp0 -= lagrange_half_filter; // adjust to first sample in window

   //***
   // Outer summation loop:
   //***
   ossimDpt3d S (0, 0, 0);
   for (ossim_uint32 j=samp0; j<(samp0+filter_size+bump); ++j)
   {
      ossim_float64 numerator   = 1.0;
      ossim_float64 denominator = 1.0;

      //***
      // Skip this sample if too close to desired time:
      //***
      if (bump && (j == (samp0+lagrange_half_filter) ))
         ++j;

      //***
      // Inner loop for product series:
      //***
      for (ossim_uint32 i=samp0; i<(samp0+filter_size+bump); ++i)
      {
         //***
         // Skip this sample if too close to desired time:
         //***
         if (bump && (i == (samp0+lagrange_half_filter) ))
            ++i;

         if (i != j)
         {
            numerator   *= time - T[i];
            denominator *= T[j] - T[i];
         }
      }

      ossimDpt3d p = V[j];
      p = p * numerator;
      p = p / denominator;
      S += p;
   }

   li = S;
}

ossim_float64 ossimQVProcSupportData::convertTimeStamp(const ossimString& time_stamp) const
{
   double ti;
   convertTimeStamp(time_stamp, ti);
   return ti;
}

void ossimQVProcSupportData::convertTimeStamp(const ossimString& time_stamp,
                                                 ossim_float64& ti) const
{
   int    year, month, day, hour, minute;
   double second;

   //***
   // Time stamps are in the format: "yyyy-mm-dd hh:mm:ss.ssssss"
   //***
   int converted = sscanf(time_stamp,
                          "%4d-%2d-%2d %2d:%2d:%9lf",
                          &year, &month, &day,
                          &hour, &minute, &second);

   if (converted != 6)
   {
      setErrorStatus();
      ti = ossim::nan();
   }
   else
   {
      ti = (((((year-2002.0)*12.0 + month - 1.0)*365.0 + day - 1.0)*24.0
             + hour)*60.0 + minute)*60.0 + second;
   }
}

void ossimQVProcSupportData::getGeoPosPoint (ossim_uint32 point,
                                                ossimDpt& ip,
                                                ossimGpt& gp) const
{
   if (point < theGeoPosImagePoints.size())
   {
      ip = theGeoPosImagePoints [point];
      gp = theGeoPosGroundPoints[point];
   }
}

void ossimQVProcSupportData::printInfo(ostream& os) const
{
   ossimString corr_att = "NO";
   if (theStarTrackerUsed)
      corr_att = "YES";

   os << "\n----------------- Info on HJ1 Image -------------------"
      << "\n  "
      << "\n  Job Number (ID):      " << theImageID
      << "\n  Acquisition Date:     " << theAcquisitionDate
      << "\n  Instrument:           " << theInstrument
      << "\n  Instrument Index:     " << theInstrumentIndex
      << "\n  Production Date:      " << theProductionDate
      << "\n  Number of Bands:      " << theNumBands
      << "\n  Geo Center Point:     " << theRefGroundPoint
      << "\n  Detector count:       " << theDetectorCount
      << "\n  Image Size:           " << theImageSize
      << "\n  Incidence Angle:      " << theIncidenceAngle
      << "\n  Viewing Angle:        " << theViewingAngle      
      << "\n  Scene Orientation:    " << theSceneOrientation 
      << "\n  Corrected Attitude:   " << corr_att
      << "\n  Sun Azimuth:          " << theSunAzimuth
      << "\n  Sun Elevation:        " << theSunElevation
      << "\n  Sub image offset:     " << theSubImageOffset
      << "\n  Step Count:           " << theStepCount
      << "\n  PixelLookAngleX size: " << thePixelLookAngleX.size()
      << "\n  thePosEcfSamples size:" << thePosEcfSamples.size()
      << "\n  Corner Points:"
      << "\n     UL: " << theUlCorner
      << "\n     UR: " << theUrCorner
      << "\n     LR: " << theLrCorner
      << "\n     LL: " << theLlCorner
      << "\n"
      << "\n---------------------------------------------------------"
      << "\n  " << std::endl;
}

ossimString ossimQVProcSupportData::getSensorID() const
{
  return theSensorID;
}

ossimString   ossimQVProcSupportData::getMetadataVersionString() const
{
   if (theMetadataVersion == OSSIM_QVProc_VERSION_1_1)
   {
      return ossimString("1.1");
   }
   else if (theMetadataVersion == OSSIM_QVProc_VERSION_1_0)
   {
      return ossimString("1.0");
   }
   return ossimString("unknown");
}

ossimString ossimQVProcSupportData::getAcquisitionDate() const
{
   return theAcquisitionDate;
}

ossimString ossimQVProcSupportData::getProductionDate() const
{
   return theProductionDate;
}

ossimString ossimQVProcSupportData::getImageID() const
{
   return theImageID;
}

ossimFilename ossimQVProcSupportData::getMetadataFile() const
{
   return theMetadataFile;
}

ossimString ossimQVProcSupportData::getInstrument() const
{
   return theInstrument;
}

ossim_uint32 ossimQVProcSupportData::getInstrumentIndex() const
{
   return theInstrumentIndex;
}

void ossimQVProcSupportData::getSunAzimuth(ossim_float64& az) const
{
   az = theSunAzimuth;
}

void ossimQVProcSupportData::getSunElevation(ossim_float64& el) const
{
   el = theSunElevation;
}

void ossimQVProcSupportData::getImageSize(ossimDpt& sz) const
{
   sz = theImageSize;
}

void ossimQVProcSupportData::getLineSamplingPeriod(ossim_float64& pe) const
{
   pe = theLineSamplingPeriod;
}


void ossimQVProcSupportData::getRefImagingTime(ossim_float64& t) const
{
	t = theReferenceTime;
}

void ossimQVProcSupportData::getRefImagingTimeLine(ossim_float64& tl) const
{
	tl = theReferenceTimeLine;
}

bool ossimQVProcSupportData::isStarTrackerUsed() const
{
   return theStarTrackerUsed;
}

bool ossimQVProcSupportData::isSwirDataUsed() const
{
   return theSwirDataFlag;
}

ossim_uint32 ossimQVProcSupportData::getNumberOfBands() const
{
   return theNumBands;
}

ossim_uint32 ossimQVProcSupportData::getStepCount() const
{
   return theStepCount;
}

void ossimQVProcSupportData::getIncidenceAngle(ossim_float64& ia) const
{
   ia = theIncidenceAngle;
}

void ossimQVProcSupportData::getViewingAngle(ossim_float64& va) const
{
   va = theViewingAngle;
}

void ossimQVProcSupportData::getSceneOrientation(ossim_float64& so) const
{
   so = theSceneOrientation;
}

void ossimQVProcSupportData::getRefGroundPoint(ossimGpt& gp) const
{
   gp = theRefGroundPoint;
}

void ossimQVProcSupportData::getRefImagePoint(ossimDpt& rp) const
{
   rp = theRefImagePoint;
}

ossim_uint32 ossimQVProcSupportData::getNumSamples() const
{
   return (ossim_uint32)theLineSamples.size();
}

ossim_uint32 ossimQVProcSupportData::getNumGeoPosPoints() const
{
   return (ossim_uint32)theGeoPosImagePoints.size();
}

void ossimQVProcSupportData::getUlCorner(ossimGpt& pt) const
{
   pt = theUlCorner;
}

void ossimQVProcSupportData::getUrCorner(ossimGpt& pt) const
{
   pt = theUrCorner;
}

void ossimQVProcSupportData::getLrCorner(ossimGpt& pt) const
{
   pt = theLrCorner;
}

void ossimQVProcSupportData::getLlCorner(ossimGpt& pt) const
{
   pt = theLlCorner;
}

void ossimQVProcSupportData::getImageRect(ossimDrect& rect)const
{
   rect = ossimDrect(0.0, 0.0, theImageSize.x-1.0, theImageSize.y-1.0);
}

void ossimQVProcSupportData::getSubImageOffset(ossimDpt& offset) const
{
   offset = theSubImageOffset;
}

bool ossimQVProcSupportData::saveState(ossimKeywordlist& kwl,
                                          const char* prefix)const
{
   kwl.add(prefix,
           ossimKeywordNames::TYPE_KW,
           "ossimQVProcSupportData",
           true);

   kwl.add(prefix,
           "metadata_file",
           theMetadataFile,
           true);

   kwl.add(prefix,
           ossimKeywordNames::AZIMUTH_ANGLE_KW,
           theSunAzimuth,
           true);

   kwl.add(prefix,
           ossimKeywordNames::ELEVATION_ANGLE_KW,
           theSunElevation,
           true);

   //---
   // Note: since this is a new keyword, use the point.toString as there is
   // no backwards compatibility issues.
   //---
   kwl.add(prefix,
           "detector_count",
           theDetectorCount,
           true);

   kwl.add(prefix,
           "image_size",
           ossimString::toString(theImageSize.x) + " " +
           ossimString::toString(theImageSize.y),
           true);

   kwl.add(prefix,
           "reference_ground_point",
           ossimString::toString(theRefGroundPoint.latd()) + " " +
           ossimString::toString(theRefGroundPoint.lond()) + " " +
           ossimString::toString(theRefGroundPoint.height()) + " " +
           theRefGroundPoint.datum()->code(),
           true);

   kwl.add(prefix,
           "reference_image_point",
           ossimString::toString(theRefImagePoint.x) + " " +
           ossimString::toString(theRefImagePoint.y),
           true);

   kwl.add(prefix,
           "sub_image_offset",
           ossimString::toString(theSubImageOffset.x) + " " +
           ossimString::toString(theSubImageOffset.y),
           true);
   
   kwl.add(prefix,
           "line_sampling_period",
           theLineSamplingPeriod,
           true);


   kwl.add(prefix,
	   "reference_time",
       theReferenceTime,
	   true);

   kwl.add(prefix,
	   "reference_time_line",
	   theReferenceTimeLine,
	   true);

   ossimString tempString;
   ossim_uint32 idx = 0;

   tempString = "";
   for(idx = 0; idx < thePixelLookAngleX.size(); ++idx)
   {
      tempString += (ossimString::toString(thePixelLookAngleX[idx]) + " ");
   }

   kwl.add(prefix,
           "pixel_lookat_angle_x",
           tempString,
           true);

   kwl.add(prefix,
           "number_of_pixel_lookat_angle_x",
           static_cast<ossim_uint32>(thePixelLookAngleX.size()),
           true);

   tempString = "";
   for(idx = 0; idx < thePixelLookAngleY.size(); ++idx)
   {
      tempString += (ossimString::toString(thePixelLookAngleY[idx]) + " ");
   }
   kwl.add(prefix,
           "pixel_lookat_angle_y",
           tempString,
           true);
   kwl.add(prefix,
           "number_of_pixel_lookat_angle_y",
           static_cast<ossim_uint32>(thePixelLookAngleY.size()),
           true);


   tempString = "";
   for(idx = 0; idx < theAttitudeSamples.size(); ++idx)
   {
      tempString += (ossimString::toString(theAttitudeSamples[idx].x) + " " +
                     ossimString::toString(theAttitudeSamples[idx].y) + " " +
                     ossimString::toString(theAttitudeSamples[idx].z) + " ");
   }
   kwl.add(prefix,
           "attitude_samples",
           tempString,
		   true);

   tempString = "";
   for(idx = 0; idx < theAttitudeBias.size(); ++idx)
   {
	   tempString += (ossimString::toString(theAttitudeBias[idx].x) + " " +
		   ossimString::toString(theAttitudeBias[idx].y) + " " +
		   ossimString::toString(theAttitudeBias[idx].z) + " ");
   }
   kwl.add(prefix,
	   "attitude_bias",
	   tempString,
	   true);

   tempString = "";
   for(idx = 0; idx < theAttitudeVelBias.size(); ++idx)
   {
	   tempString += (ossimString::toString(theAttitudeVelBias[idx].x) + " " +
		   ossimString::toString(theAttitudeVelBias[idx].y) + " " +
		   ossimString::toString(theAttitudeVelBias[idx].z) + " ");
   }
   kwl.add(prefix,
	   "attitude_velocity_bias",
	   tempString,
	   true);

   tempString = "";
   for(idx = 0; idx < theAttitudeVelSamples.size(); ++idx)
   {
	   tempString += (ossimString::toString(theAttitudeVelSamples[idx].x) + " " +
		   ossimString::toString(theAttitudeVelSamples[idx].y) + " " +
		   ossimString::toString(theAttitudeVelSamples[idx].z) + " ");
   }
   kwl.add(prefix,
	   "attitude_velocity_samples",
	   tempString,
	   true);
   kwl.add(prefix,
           "number_of_attitude_samples",
           static_cast<ossim_uint32>(theAttitudeSamples.size()),
           true);

   tempString = "";
   for(idx = 0; idx < theAttSampTimes.size(); ++idx)
   {
      tempString += (ossimString::toString(theAttSampTimes[idx]) + " ");
   }
   kwl.add(prefix,
           "attitude_sample_times",
           tempString,
           true);
   kwl.add(prefix,
           "number_of_attitude_sample_times",
           static_cast<ossim_uint32>(theAttSampTimes.size()),
           true);

   tempString = "";
   for(idx = 0; idx < theSampTimesBias.size(); ++idx)
   {
      tempString += (ossimString::toString(theSampTimesBias[idx]) + " ");
   }
   kwl.add(prefix,
           "sample_times_bias",
           tempString,
           true);

   tempString = "";
   for(idx = 0; idx < thePosEcfSamples.size(); ++idx)
   {
      tempString += (ossimString::toString(thePosEcfSamples[idx].x) + " " +
                     ossimString::toString(thePosEcfSamples[idx].y) + " " +
                     ossimString::toString(thePosEcfSamples[idx].z) + " ");
   }
   kwl.add(prefix,
           "position_ecf_samples",
           tempString,
           true);
   kwl.add(prefix,
           "number_of_position_ecf_samples",
           static_cast<ossim_uint32>(thePosEcfSamples.size()),
           true);

   tempString = "";
   for(idx = 0; idx < theVelEcfSamples.size(); ++idx)
   {
      tempString += (ossimString::toString(theVelEcfSamples[idx].x) + " " +
                     ossimString::toString(theVelEcfSamples[idx].y) + " " +
                     ossimString::toString(theVelEcfSamples[idx].z) + " ");
   }
   kwl.add(prefix,
           "velocity_ecf_samples",
           tempString,
           true);
   kwl.add(prefix,
           "number_of_velocity_ecf_samples",
           static_cast<ossim_uint32>(thePosEcfSamples.size()),
           true);

   tempString = "";
   for(idx = 0; idx < theEphSampTimes.size(); ++idx)
   {
	   tempString += (ossimString::toString(theEphSampTimes[idx]) + " ");
   }
   kwl.add(prefix,
	   "eph_sample_times",
	   tempString,
	   true);
   kwl.add(prefix,
	   "number_of_eph_sample_times",
	   static_cast<ossim_uint32>(theEphSampTimes.size()),
	   true);


   tempString = "";
   for(idx = 0; idx < theLineSampTimes.size(); ++idx)
   {
	   tempString += (ossimString::toString(theLineSampTimes[idx]) + " ");
   }
   kwl.add(prefix,
	   "line_sample_times",
	   tempString,
	   true);
   kwl.add(prefix,
	   "number_of_line_sample_times",
	   static_cast<ossim_uint32>(theLineSampTimes.size()),
	   true);

   tempString = "";
   for(idx = 0; idx < theLineSamples.size(); ++idx)
   {
	   tempString += (ossimString::toString(theLineSamples[idx]) + " ");
   }
   kwl.add(prefix,
	   "line_samples",
	   tempString,
	   true);
   kwl.add(prefix,
	   "number_of_line_samples",
	   static_cast<ossim_uint32>(theLineSamples.size()),
	   true);

   kwl.add(prefix,
           "star_tracker_used_flag",
           static_cast<ossim_uint32>(theStarTrackerUsed),
           true);

   kwl.add(prefix,
           "swir_data_flag",
           static_cast<ossim_uint32>(theSwirDataFlag),
           true);

   kwl.add(prefix,
           ossimKeywordNames::NUMBER_BANDS_KW,
           theNumBands,
           true);

   kwl.add(prefix,
           "image_id",
           theImageID,
           true);

   kwl.add(prefix,
           "instrument",
           theInstrument,
           true);

   kwl.add(prefix,
           "instrument_index",
           theInstrumentIndex,
           true);

   kwl.add(prefix,
           ossimKeywordNames::IMAGE_DATE_KW,
           theAcquisitionDate,
           true);

   kwl.add(prefix,
           "production_date",
           theProductionDate,
           true);

   kwl.add(prefix,
           "incident_angle",
           theIncidenceAngle,
           true);

   kwl.add(prefix,
           "viewing_angle",
           theViewingAngle,
           true);

   kwl.add(prefix,
           "scene_orientation",
           theSceneOrientation,
           true);
                      
   kwl.add(prefix,
           "step_count",
           theStepCount,
           true);
           
   kwl.add(prefix,
           "ul_ground_point",
           ossimString::toString(theUlCorner.latd()) + " " +
           ossimString::toString(theUlCorner.lond()) + " " +
           ossimString::toString(theUlCorner.height()) + " " +
           theUlCorner.datum()->code(),
           true);

   kwl.add(prefix,
           "ur_ground_point",
           ossimString::toString(theUrCorner.latd()) + " " +
           ossimString::toString(theUrCorner.lond()) + " " +
           ossimString::toString(theUrCorner.height()) + " " +
           theUrCorner.datum()->code(),
           true);

   kwl.add(prefix,
           "lr_ground_point",
           ossimString::toString(theLrCorner.latd()) + " " +
           ossimString::toString(theLrCorner.lond()) + " " +
           ossimString::toString(theLrCorner.height()) + " " +
           theLrCorner.datum()->code(),
           true);

   kwl.add(prefix,
           "ll_ground_point",
           ossimString::toString(theLlCorner.latd()) + " " +
           ossimString::toString(theLlCorner.lond()) + " " +
           ossimString::toString(theLlCorner.height()) + " " +
           theLlCorner.datum()->code(),
           true);

   kwl.add(prefix,
           "sensorID",
           theSensorID,
           true);


   tempString = "";
   for(idx = 0; idx < thePhysicalBias.size(); ++idx)
   {
     tempString += (ossimString::toString(thePhysicalBias[idx]) + " ");
   }
   kwl.add(prefix,
           "physical_bias",
           tempString,
           true);

   tempString = "";
   for(idx = 0; idx < thePhysicalGain.size(); ++idx)
   {
     tempString += (ossimString::toString(thePhysicalGain[idx]) + " ");
   }
   kwl.add(prefix,
           "physical_gain",
           tempString,
           true);

   tempString = "";
   for(idx = 0; idx < theSolarIrradiance.size(); ++idx)
   {
     tempString += (ossimString::toString(theSolarIrradiance[idx]) + " ");
   }

   kwl.add(prefix,
           "solar_irradiance",
           tempString,
           true);

   return true;
}

bool ossimQVProcSupportData::loadState(const ossimKeywordlist& kwl,
                                          const char* prefix)
{
   clearFields();

   ossimString type = kwl.find(prefix, ossimKeywordNames::TYPE_KW);

   if(type != "ossimQVProcSupportData")
   {
      return false;
   }
   theMetadataFile = kwl.find(prefix, "metadata_file");

   theSunAzimuth   = ossimString(kwl.find(prefix, ossimKeywordNames::AZIMUTH_ANGLE_KW)).toDouble();
   theSunElevation = ossimString(kwl.find(prefix, ossimKeywordNames::ELEVATION_ANGLE_KW)).toDouble();

   const char* lookup = kwl.find(prefix, "detector_count");
   if (lookup)
   {
      theDetectorCount = ossimString(lookup).toUInt32();
   }

   theImageSize      = createDpt(kwl.find(prefix, "image_size"));
   theRefGroundPoint = createGround(kwl.find(prefix, "reference_ground_point"));
   theRefImagePoint  = createDpt(kwl.find(prefix, "reference_image_point"));
   theSubImageOffset = createDpt(kwl.find(prefix, "sub_image_offset"));
   
   theLineSamplingPeriod = ossimString(kwl.find(prefix, "line_sampling_period")).toDouble();

   theReferenceTime = ossimString(kwl.find(prefix, "reference_time")).toDouble();
   theReferenceTimeLine = ossimString(kwl.find(prefix, "reference_time_line")).toDouble();

   ossim_uint32 idx = 0;
   ossim_uint32 total =  ossimString(kwl.find(prefix,"number_of_pixel_lookat_angle_x")).toUInt32();
   ossimString tempString;

   thePixelLookAngleX.resize(total);
   tempString = kwl.find(prefix,"pixel_lookat_angle_x");
   if(tempString != "")
   {
      std::istringstream in(tempString.string());
      ossimString tempValue;
      for(idx = 0; idx < thePixelLookAngleX.size();++idx)
      {
         in >> tempValue.string();
         thePixelLookAngleX[idx] = tempValue.toDouble();
      }
   }

   total =  ossimString(kwl.find(prefix,"number_of_pixel_lookat_angle_y")).toUInt32();
   thePixelLookAngleY.resize(total);
   tempString = kwl.find(prefix,"pixel_lookat_angle_y");
   if(tempString != "")
   {
      std::istringstream in(tempString.string());
      ossimString tempValue;
      for(idx = 0; idx < thePixelLookAngleY.size();++idx)
      {
         in >> tempValue.string();
         thePixelLookAngleY[idx] = tempValue.toDouble();
      }
   }

   total =  ossimString(kwl.find(prefix,"number_of_attitude_samples")).toUInt32();
   theAttitudeSamples.resize(total);
   tempString = kwl.find(prefix,"attitude_samples");
   if(tempString != "")
   {
      std::istringstream in(tempString.string());
      ossimString x, y, z;
      for(idx = 0; idx < theAttitudeSamples.size();++idx)
      {
         in >> x.string() >> y.string() >> z.string();
         theAttitudeSamples[idx] =ossimDpt3d(x.toDouble(), y.toDouble(), z.toDouble());
      }
   }

   theAttitudeVelSamples.resize(total);
   tempString = kwl.find(prefix,"attitude_velocity_samples");
   if(tempString != "")
   {
	   std::istringstream in(tempString.string());
	   ossimString x, y, z;
	   for(idx = 0; idx < theAttitudeVelSamples.size();++idx)
	   {
		   in >> x.string() >> y.string() >> z.string();
		   theAttitudeVelSamples[idx] =ossimDpt3d(x.toDouble(), y.toDouble(), z.toDouble());
	   }
   }

   theAttitudeBias.resize(total);
   tempString = kwl.find(prefix,"attitude_bias");
   if(tempString != "")
   {
	   std::istringstream in(tempString.string());
	   ossimString x, y, z;
	   for(idx = 0; idx < theAttitudeBias.size();++idx)
	   {
		   in >> x.string() >> y.string() >> z.string();
		   theAttitudeBias[idx] =ossimDpt3d(x.toDouble(), y.toDouble(), z.toDouble());
	   }
   }

   theAttitudeVelBias.resize(total);
   tempString = kwl.find(prefix,"attitude_velocity_bias");
   if(tempString != "")
   {
	   std::istringstream in(tempString.string());
	   ossimString x, y, z;
	   for(idx = 0; idx < theAttitudeVelBias.size();++idx)
	   {
		   in >> x.string() >> y.string() >> z.string();
		   theAttitudeVelBias[idx] =ossimDpt3d(x.toDouble(), y.toDouble(), z.toDouble());
	   }
   }

   total =  ossimString(kwl.find(prefix,"number_of_attitude_sample_times")).toUInt32();
   theAttSampTimes.resize(total);
   tempString = kwl.find(prefix,"attitude_sample_times");
   if(tempString != "")
   {
      std::istringstream in(tempString.string());
      ossimString tempValue;
      for(idx = 0; idx < theAttSampTimes.size();++idx)
      {
         in >> tempValue.string();
         theAttSampTimes[idx] = tempValue.toDouble();
      }
   }

   theSampTimesBias.resize(total);
   tempString = kwl.find(prefix,"sample_times_bias");
   if(tempString != "")
   {
	   std::istringstream in(tempString.string());
	   ossimString tempValue;
	   for(idx = 0; idx < theSampTimesBias.size();++idx)
	   {
		   in >> tempValue.string();
		   theSampTimesBias[idx] = tempValue.toDouble();
	   }
   }

   total =  ossimString(kwl.find(prefix,"number_of_position_ecf_samples")).toUInt32();
   thePosEcfSamples.resize(total);
   tempString = kwl.find(prefix,"position_ecf_samples");
   if(tempString != "")
   {
      std::istringstream in(tempString.string());
      ossimString x, y, z;
      for(idx = 0; idx < thePosEcfSamples.size();++idx)
      {
         in >> x.string() >> y.string() >> z.string();
         thePosEcfSamples[idx] = ossimDpt3d(x.toDouble(), y.toDouble(), z.toDouble());
      }
   }

   total =  ossimString(kwl.find(prefix,"number_of_velocity_ecf_samples")).toUInt32();
   theVelEcfSamples.resize(total);
   tempString = kwl.find(prefix,"velocity_ecf_samples");
   if(tempString != "")
   {
      std::istringstream in(tempString.string());
      ossimString x, y, z;
      for(idx = 0; idx < theVelEcfSamples.size();++idx)
      {
         in >> x.string() >> y.string() >> z.string();
         theVelEcfSamples[idx] = ossimDpt3d(x.toDouble(), y.toDouble(), z.toDouble());
      }
   }


   total =  ossimString(kwl.find(prefix,"number_of_eph_sample_times")).toUInt32();
   theEphSampTimes.resize(total);
   tempString = kwl.find(prefix,"eph_sample_times");
   if(tempString != "")
   {
	   std::istringstream in(tempString.string());
	   ossimString tempValue;
	   for(idx = 0; idx < theEphSampTimes.size();++idx)
	   {
		   in >> tempValue.string();
		   theEphSampTimes[idx] = tempValue.toDouble();
	   }
   }

   total =  ossimString(kwl.find(prefix,"number_of_line_sample_times")).toUInt32();
   theLineSampTimes.resize(total);
   tempString = kwl.find(prefix,"line_sample_times");
   if(tempString != "")
   {
	   std::istringstream in(tempString.string());
	   ossimString tempValue;
	   for(idx = 0; idx < theLineSampTimes.size();++idx)
	   {
		   in >> tempValue.string();
		   theLineSampTimes[idx] = tempValue.toDouble();
	   }
   }

   total =  ossimString(kwl.find(prefix,"number_of_line_samples")).toUInt32();
   theLineSamples.resize(total);
   tempString = kwl.find(prefix,"line_samples");
   if(tempString != "")
   {
	   std::istringstream in(tempString.string());
	   ossimString tempValue;
	   for(idx = 0; idx < theLineSamples.size();++idx)
	   {
		   in >> tempValue.string();
		   theLineSamples[idx] = tempValue.toDouble();
	   }
   }

   theStarTrackerUsed = ossimString(kwl.find(prefix, "star_tracker_used_flag")).toBool();
   theSwirDataFlag    = ossimString(kwl.find(prefix, "swir_data_flag")).toBool();
   theNumBands        = ossimString(kwl.find(prefix, ossimKeywordNames::NUMBER_BANDS_KW)).toUInt32();
   theAcquisitionDate = kwl.find(prefix, ossimKeywordNames::IMAGE_DATE_KW);
   theProductionDate  = kwl.find(prefix, "production_date");
   theImageID         = kwl.find(prefix, "image_id");
   theInstrument      = kwl.find(prefix, "instrument");
   theInstrumentIndex = ossimString(kwl.find(prefix, "instrument_index")).toUInt32();
   theStepCount       = ossimString(kwl.find(prefix, "step_count")).toInt32();
   
   theIncidenceAngle  = ossimString(kwl.find(prefix, "incident_angle")).toDouble();
   theViewingAngle    = ossimString(kwl.find(prefix, "viewing_angle")).toDouble();
   theSceneOrientation= ossimString(kwl.find(prefix, "scene_orientation")).toDouble();
   
   theUlCorner =createGround( kwl.find(prefix, "ul_ground_point"));
   theUrCorner =createGround( kwl.find(prefix, "ur_ground_point"));
   theLrCorner =createGround( kwl.find(prefix, "lr_ground_point"));
   theLlCorner =createGround( kwl.find(prefix, "ll_ground_point"));

   theSensorID = ossimString(kwl.find(prefix, "sensorID"));

   thePhysicalBias.resize(theNumBands);
   tempString = kwl.find(prefix,"physical_bias");
   if(tempString != "")
   {
      std::istringstream in(tempString.string());
      ossimString tempValue;
      for(idx = 0; idx < thePhysicalBias.size();++idx)
      {
         in >> tempValue.string();
         thePhysicalBias[idx] = tempValue.toDouble();
      }
   }

   thePhysicalGain.resize(theNumBands);
   tempString = kwl.find(prefix,"physical_gain");
   if(tempString != "")
   {
      std::istringstream in(tempString.string());
      ossimString tempValue;
      for(idx = 0; idx < thePhysicalGain.size();++idx)
      {
         in >> tempValue.string();
         thePhysicalGain[idx] = tempValue.toDouble();
      }
   }

   theSolarIrradiance.resize(theNumBands);
   tempString = kwl.find(prefix,"solar_irradiance");
   if(tempString != "")
   {
      std::istringstream in(tempString.string());
      ossimString tempValue;
      for(idx = 0; idx < theSolarIrradiance.size();++idx)
      {
         in >> tempValue.string();
         theSolarIrradiance[idx] = tempValue.toDouble();
      }
   }

   return true;
}

ossimGpt ossimQVProcSupportData::createGround(const ossimString& s)const
{
   std::istringstream in(s.string());
   ossimString lat, lon, height;
   ossimString code;

   in >> lat.string() >> lon.string() >> height.string() >> code.string();

   return ossimGpt(lat.toDouble(),
                   lon.toDouble(),
                   height.toDouble(),
                   ossimDatumFactory::instance()->create(code));

}

ossimDpt ossimQVProcSupportData::createDpt(const ossimString& s)const
{
   std::istringstream in(s.string());
   ossimString x, y;
   ossimString code;

   in >> x.string() >> y.string();

   return ossimDpt(x.toDouble(), y.toDouble());

}

bool ossimQVProcSupportData::initMetadataVersion(ossimRefPtr<ossimXmlDocument> xmlDocument)
{
   ossimString xpath;
   std::vector<ossimRefPtr<ossimXmlNode> > xml_nodes;

   //---
   // Get the version string which can be used as a key for parsing.
   //---
   xpath = "/DATASET/BASE_INFO/METADATA_FORMAT";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() == 0)
   {
      setErrorStatus();
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_DEBUG)
            << "DEBUG:\nCould not find: " << xpath
            << endl;
      }
      return false;
   }

   ossimString attribute = "version";
   ossimString value;
   xml_nodes[0]->getAttributeValue(value, attribute);
   if (value == "1.0")
   {
      theMetadataVersion = OSSIM_QVProc_VERSION_1_0;
   }
   else if (value == "1.1")
   {
      theMetadataVersion = OSSIM_QVProc_VERSION_1_1;
   }

   if (theMetadataVersion == OSSIM_QVProc_VERSION_UNKNOWN)
   {
      setErrorStatus();
      if (traceDebug())
      {
         ossimNotify(ossimNotifyLevel_WARN)
            << "WARNING:  metadata version not found!"
            << std::endl;
      }
      return false;
   }
   return true;
}

bool ossimQVProcSupportData::initImageId(
   ossimRefPtr<ossimXmlDocument> xmlDocument)
{
   ossimString xpath;
   vector<ossimRefPtr<ossimXmlNode> > xml_nodes;

   //---
   // Fetch the Image ID:
   //---
   xpath = "/DATASET/BASE_INFO/JOB_ID";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() == 0)
   {
      setErrorStatus();
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_DEBUG)
            << "DEBUG:\nCould not find: " << xpath
            << endl;
      }
      return false;
   }
   theImageID = xml_nodes[0]->getText();
   return true;
}

bool ossimQVProcSupportData::initSceneSource(
   ossimRefPtr<ossimXmlDocument> xmlDocument)
{
   ossimString xpath;
   vector<ossimRefPtr<ossimXmlNode> > xml_nodes;

  //---
  // Fetch the mission index (Spot 1, 4 or 5):
  // and generate theSensorID
  //---
   xml_nodes.clear();
   xpath = "/DATASET/BASE_INFO/Scene_Source";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() == 0)
   {
     setErrorStatus();
     if(traceDebug())
       {
	 ossimNotify(ossimNotifyLevel_DEBUG)
	   << "DEBUG:\nCould not find: " << xpath
	   << std::endl;
       }
     return false;
   }
   if (xml_nodes[0]->getText() == "1")
     theSensorID = "Spot 1";
   if (xml_nodes[0]->getText() == "2")
     theSensorID = "Spot 2";
   if (xml_nodes[0]->getText() == "4")
     theSensorID = "Spot 4";
   if (xml_nodes[0]->getText() == "5")
     theSensorID = "Spot 5";

   //---
   // Fetch the Sun Azimuth:
   //---
   xml_nodes.clear();
   xpath = "/Dimap_Document/Dataset_Sources/Source_Information/Scene_Source/SUN_AZIMUTH";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() == 0)
   {
      setErrorStatus();
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_DEBUG)
            << "DEBUG:\nCould not find: " << xpath
            << std::endl;
      }
      return false;
   }
   theSunAzimuth = xml_nodes[0]->getText().toDouble();

   //---
   // Fetch the Sun Elevation:
   //---
   xml_nodes.clear();
   xpath = "/Dimap_Document/Dataset_Sources/Source_Information/Scene_Source/SUN_ELEVATION";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() == 0)
   {
      setErrorStatus();
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_DEBUG)
            << "DEBUG:\nCould not find: " << xpath
            << std::endl;
      }
      return false;
   }
   theSunElevation = xml_nodes[0]->getText().toDouble();

   //---
   // Fetch incidence angle:
   //---
   xml_nodes.clear();
   xpath = "/Dimap_Document/Dataset_Sources/Source_Information/Scene_Source/INCIDENCE_ANGLE";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() == 0)
   {
      setErrorStatus();
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_DEBUG)
            << "DEBUG:\nCould not find: " << xpath
            << std::endl;
      }
      return false;
   }
   theIncidenceAngle = xml_nodes[0]->getText().toDouble();

   //---
   // Fetch viewing angle:
   //
   // From the SPOT Dimap documentation (Dimap Generic 1.0), VIEWING_ANGLE
   // (the scene instrumental viewing angle) is ONLY available for SPOT5 data.
   // FROM SPOT: You can use use incidence angle to calculate viewing angle
   // (called look direction as well).
   // FIX (see below): need theSatelliteAltitude and theIncidenceAngle. The
   // equation is shown below where RT is the mean value of WGS84 ellipsoid 
   // semi-axis.
   //---
   if(this->theSensorID == "Spot 5")
   {
      xml_nodes.clear();
      xpath = "/Dimap_Document/Dataset_Sources/Source_Information/Scene_Source/VIEWING_ANGLE";
      xmlDocument->findNodes(xpath, xml_nodes);
      if (xml_nodes.size() == 0)
      {
         setErrorStatus();
         if(traceDebug())
         {
            ossimNotify(ossimNotifyLevel_DEBUG)
               << "DEBUG:\nCould not find: " << xpath
               << std::endl;
         }
         return false;
      }
      theViewingAngle = xml_nodes[0]->getText().toDouble();
   }
   else
   {
      xml_nodes.clear();
      xpath = "/Dimap_Document/Data_Strip/Ephemeris/SATELLITE_ALTITUDE";
      
      theViewingAngle = -1.0;
      xmlDocument->findNodes(xpath, xml_nodes);
      if (xml_nodes.size() == 0)
      {
         setErrorStatus();
         if(traceDebug())
         {
            ossimNotify(ossimNotifyLevel_DEBUG)
               << "DEBUG:\nCould not find: " << xpath
               << std::endl;
         }
         return false;
      }
      //compute VIEWING_ANGLE
      double theSatelliteAltitude =  xml_nodes[0]->getText().toDouble();
      double RT = 63710087714.0;
      theViewingAngle = asin((RT/(RT+theSatelliteAltitude))*sin(theIncidenceAngle));
   }

   //---
   // Fetch Step Count:
   //---
   xml_nodes.clear();
   xpath = "/Dimap_Document/Data_Strip/Sensor_Configuration/Mirror_Position/STEP_COUNT";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() == 0)
   {
      setErrorStatus();
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_DEBUG)
            << "DEBUG:\nCould not find: " << xpath
            << std::endl;
      }
      return false;
   }
   theStepCount = xml_nodes[0]->getText().toInt();
   
   return true;
}

bool ossimQVProcSupportData::initFramePoints(
   ossimRefPtr<ossimXmlDocument> xmlDocument)
{
   ossimString xpath;
   vector<ossimRefPtr<ossimXmlNode> > xml_nodes;

   //---
   // Corner points:
   //---
   xml_nodes.clear();
   xpath = "/Dimap_Document/Dataset_Frame/Vertex";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() != 4)
   {
      setErrorStatus();
      return false;
   }
   std::vector<ossimRefPtr<ossimXmlNode> >::iterator node = xml_nodes.begin();
   while (node != xml_nodes.end())
   {
      ossimGpt gpt;
      ossimDpt ipt;

      std::vector<ossimRefPtr<ossimXmlNode> > sub_nodes;
      xpath = "FRAME_LAT";
      (*node)->findChildNodes(xpath, sub_nodes);
      if (sub_nodes.size() == 0)
      {
         setErrorStatus();
         return false;
      }
      gpt.lat = sub_nodes[0]->getText().toDouble();

      sub_nodes.clear();
      xpath = "FRAME_LON";
      (*node)->findChildNodes(xpath, sub_nodes);
      if (sub_nodes.size() == 0)
      {
         setErrorStatus();
         return false;
      }
      gpt.lon = sub_nodes[0]->getText().toDouble();
      gpt.hgt = 0.0; // assumed

      sub_nodes.clear();
      xpath = "FRAME_ROW";
      (*node)->findChildNodes(xpath, sub_nodes);
      if (sub_nodes.size() == 0)
      {
         setErrorStatus();
         return false;
      }
      ipt.line = sub_nodes[0]->getText().toDouble() - 1.0;

      sub_nodes.clear();
      xpath = "FRAME_COL";
      (*node)->findChildNodes(xpath, sub_nodes);
      if (sub_nodes.size() == 0)
      {
         setErrorStatus();
         return false;
      }
      ipt.samp = sub_nodes[0]->getText().toDouble() - 1.0;

      if (ipt.line < 1.0)
         if (ipt.samp < 1.0)
            theUlCorner = gpt;
         else
            theUrCorner = gpt;
      else
         if (ipt.samp < 1.0)
            theLlCorner = gpt;
         else
            theLrCorner = gpt;

      ++node;
   }

   //---
   // Center of frame.
   //---
   theRefGroundPoint.hgt = 0.0; // needs to be looked up

   xml_nodes.clear();
   xpath = "/Dimap_Document/Dataset_Frame/Scene_Center/FRAME_LON";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() != 1)
   {
      setErrorStatus();
      return false;
   }
   theRefGroundPoint.lon = xml_nodes[0]->getText().toDouble();

   xml_nodes.clear();
   xpath = "/Dimap_Document/Dataset_Frame/Scene_Center/FRAME_LAT";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() != 1)
   {
      setErrorStatus();
      return false;
   }
   theRefGroundPoint.lat = xml_nodes[0]->getText().toDouble();

  
   //---
   // Fetch scene orientation:
   //---
   xml_nodes.clear();
   xpath = "/Dimap_Document/Dataset_Frame/SCENE_ORIENTATION";
   xmlDocument->findNodes(xpath, xml_nodes);
   if (xml_nodes.size() == 0)
   {
      setErrorStatus();
      if(traceDebug())
      {
         ossimNotify(ossimNotifyLevel_DEBUG)
            << "DEBUG:\nCould not find: " << xpath
            << std::endl;
      }
      return false;
   }
   theSceneOrientation = xml_nodes[0]->getText().toDouble();  

   return true;
}

}