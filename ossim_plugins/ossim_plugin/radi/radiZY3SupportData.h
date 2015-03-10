//*******************************************************************
//
// License:  See top level LICENSE.txt file.
//
//
// Author:  Long Tengfei
//
// Description:
//
// Contains declaration of class radiZY3SupportData.
//
//*****************************************************************************
// $Id: radiZY3SupportData.h $
#ifndef radiZY3SupportData_HEADER
#define radiZY3SupportData_HEADER

#include <vector>
#include <iostream>

#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimObject.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimErrorStatusInterface.h>
#include <ossim/base/ossimString.h>
#include <ossim/base/ossimFilename.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/base/ossimDpt3d.h>
#include <ossim/base/ossimDrect.h>
#include <ossim/base/ossimGpt.h>
#include <ossim/base/ossimEcefPoint.h>

#include <ossimPluginConstants.h>
#include <armadillo>

class ossimKeywordlist;
// class ossimRefPtr;
class ossimXmlDocument;

namespace ossimplugins
{

class OSSIM_PLUGINS_DLL radiZY3SupportData : public ossimObject,
                                  public ossimErrorStatusInterface
{
public:

   /** QVProc format version */
   enum ossimQVProcVersion
   {
      OSSIM_QVProc_VERSION_UNKNOWN = 0,
      OSSIM_QVProc_VERSION_1_0 = 1,
      OSSIM_QVProc_VERSION_1_1 = 2
   };


   typedef struct _tagBL
   {
	   float		longitude;
	   float		latitude;
   }LONLAT_POS;

   typedef struct _tagTimeInfo
   {
	   int			year;
	   int			mon;
	   int			day;
	   int			hour;
	   int			min;
	   int			sec;
	   int			nsec;
   }_TIMESTAMP;

   struct SAT_POS
   {
	   float x;
	   float y;
	   float z;
	   float vx;
	   float vy;
	   float vz;
   };  
   struct SAT_ATT
   {
	   float roll;
	   float pitch;
	   float yaw;
	   float vroll;
	   float vpitch;
	   float vyaw;
   };

   typedef struct 
   {
	   char  station_id[16];		//	���ո����ݵĵ���վ�ı�ʶ
	   char	 satellite_id[16];	    //	��Ӧ�����Ǳ�ʶ
	   char   sensor_id[16];		//	��Ӧ�Ĵ�������ʶ
	   char	 work_mode[16];		//	��Ӧ�Ĺ���ģʽ��Ϣ
	   char	 JobTaskID [20];	    //	�����ʶ
	   int	 orbit_num;   		//  �����
	   int	 channel_num;	 	//  ͨ����
	   int	 totalfiles;	 	    //  �����񵥴�������ļ�����
	   int	 files_num;	 	    //  ���δ�����ļ������������е��ļ����
	   int	 desample_num;	    //	���п�������ʱ�Ľ�������,��һ���Ľ�������
	   int	 data_width;			//	��һ���������Ŀ��,(����������������Ļ�),������һ����С
	   int	 sample_bit_count;	//	���Ӵ�����ͼ��������������ֻ��8Bit��16Bit�������
	   int	 gray_image_flag;	//	����ֵΪ��ʱ����ͼ��Ϊ��ͨ���Ҷ�ͼ�񣬷���Ϊ��ͨ��α��ɫͼ��
	   _TIMESTAMP		start_time;	//	���ݽ��յĿ�ʼʱ��
	   _TIMESTAMP		end_time;		//	���ݽ��յĽ���ʱ��
	   LONLAT_POS		station_pos;	//	��Ӧ�ĵ���վ�ڵ����ϵ�λ����Ϣ	����վ��λ��Ϣ

	   int  sample_num;
	   int  line_num;
	   int  band_num;
   } QUIVIMAGE_HEAD_INFO;

   typedef struct _tagQuivAuxInfo
   {
	   int		   valid_flag;		//	��ֵΪ=1ʱ��Ч����������¸�����ϢΪ��Чֵ
	   LONLAT_POS		nadir_pos;	//	��Ӧ�����µ��λ��
	   LONLAT_POS		line_left_pos;//	�þ����߶�Ӧ������λ�ã����Ⱥ�γ�ȣ�
	   LONLAT_POS		line_right_pos;	//	�þ����߶�Ӧ���Ҳ��λ�ã����Ⱥ�γ�ȣ�
	   SAT_POS			satpos;     //���ǵ�x,y,z�����Լ�vx,vy,vz
	   SAT_ATT			satatt;     //���ǵ���̬����,��������ƫ����r,p,y
	   unsigned short line_count;
	   _TIMESTAMP		line_time;			//	����������ʱ����Ϣ
	   double      line_num;
	   double		att_time;			//	��̬���ݵ�ʱ����Ϣ
	   double		gps_time;			//	GPS��λ����ʱ��
	   double      star_time;			//  ���������ʱ����
	   double      gps_c_time;			//  GPS��ʱ��
   }QUIVIMAGE_AUX_INFO;

   radiZY3SupportData();


   radiZY3SupportData(const radiZY3SupportData& rhs);
   radiZY3SupportData(const ossimFilename& dimapFile,
                             bool  processSwir=false);

   virtual ossimObject* dup()const;

   void clearFields();
   bool parseQVProcFile(const ossimFilename& file,
	   bool processSwir=false);
   //bool loadXmlFile(const ossimFilename& file,
   //                 bool processSwir=false);

   ossimString   getSpacecraftID()                        const;
   ossimString   getSensorID()                            const;
   ossimString   getStationID()							  const;
   ossimString   getMetadataVersionString()               const;
   ossimString   getAcquisitionDate()                     const;
   ossimString   getProductionDate()                      const;
   ossimString   getImageID()                             const;
   ossimString   getInstrument()                          const;
   ossim_uint32  getInstrumentIndex()                     const;
   ossimFilename getMetadataFile()                        const;
   void          getSunAzimuth(ossim_float64& az)         const;
   void          getSunElevation(ossim_float64& el)       const;
   void          getImageSize(ossimDpt& sz)               const;
   void          getLineSamplingPeriod(ossim_float64& pe) const;
   void          getRefImagingTime(ossim_float64& t) const;
   void          getRefImagingTimeLine(ossim_float64& tl) const;
   void          getIncidenceAngle(ossim_float64& ia)     const;
   void          getViewingAngle(ossim_float64& va)       const;
   void          getSceneOrientation(ossim_float64& so)   const;
   ossim_uint32  getNumberOfBands()                       const;
   ossim_uint32  getStepCount()                           const;
   bool          isStarTrackerUsed()                      const;
   bool          isSwirDataUsed()                         const;

   //---
   // Image center point:
   //---

   /** Center of frame, sub image if there is one. */
   void getRefGroundPoint(ossimGpt& gp)         const;

   /** zero base center point */
   void getRefImagePoint(ossimDpt& rp)          const;
   
   /** Zero based image rectangle, sub image if there is one. */
   void getImageRect(ossimDrect& rect)const;

   //---
   // Sub image offset:
   //---
   void getSubImageOffset(ossimDpt& offset) const;

   //---
   // Ephemeris (m & m/s):
   //---
   void getPositionEcf(ossim_uint32 sample, ossimEcefPoint& pe) const;
   void getPositionEcf(const ossim_float64& time, ossimEcefPoint& pe) const;
   void getVelocityEcf(ossim_uint32 sample, ossimEcefPoint& ve) const;
   void getVelocityEcf(const ossim_float64& time, ossimEcefPoint& ve) const;
   void getEphSampTime(ossim_uint32 sample, ossim_float64& et) const;

   ossim_uint32 getNumEphSamples() const;

   //---
   // Attitude Angles in RADIANS:
   //---
   void getAttitude(ossim_uint32 sample, ossimDpt3d& at)  const;
   void getAttitude(const ossim_float64& time, ossimDpt3d& at) const;
   void getAttSampTime(ossim_uint32 sample, ossim_float64& at)  const;
   ossim_uint32 getNumSamples() const;


   void getLineTime(const ossim_float64& line,
	   ossim_float64& time)  const;

   //---
   // Pixel Pointing/Mirror tilt  Angles in RADIANS:
   //---
   void getPixelLookAngleX (ossim_uint32 sample, ossim_float64& pa) const;
   void getPixelLookAngleX (const ossim_float64& sample,
                            ossim_float64& pa) const;
   void getPixelLookAngleY (ossim_uint32 sample, ossim_float64& pa) const;
   void getPixelLookAngleY (const ossim_float64& sample,
                            ossim_float64& pa) const;

   //---
   // Geoposition points provided in the file (most likely just corner points):
   //---
   ossim_uint32 getNumGeoPosPoints() const;
   void getGeoPosPoint (ossim_uint32 point, ossimDpt& ip, ossimGpt& gp) const;

   //---
   // Corner points:
   //---
   void getUlCorner(ossimGpt& pt) const;
   void getUrCorner(ossimGpt& pt) const;
   void getLrCorner(ossimGpt& pt) const;
   void getLlCorner(ossimGpt& pt) const;

   //---
   // Convenient method to print important image info:
   //---
   void  printInfo (ostream& os) const;

   virtual bool saveState(ossimKeywordlist& kwl,
                          const char* prefix = 0)const;
   virtual bool loadState(const ossimKeywordlist& kwl,
	   const char* prefix = 0);

   std::vector<ossimDpt3d>     theAttitudeBias; // x=pitch, y=roll, z=yaw
   std::vector<ossimDpt3d>     theAttitudeVelBias; // x=pitch/s, y=roll/s, z=yaw/s
   std::vector<double>         theSampTimesBias;
   //ossim_float64    getSampTime(int idx)const {return theAttSampTimes[idx]+theSampTimesBias[idx]*1e-3;}
   ossimDpt3d    getAttitude(int idx)const {return theAttitudeSamples[idx]+theAttitudeBias[idx]*1e-2;}
   ossimDpt3d    getAttitudeVel(int idx)const {return theAttitudeVelSamples[idx]+theAttitudeVelBias[idx]*1e-2;}
   std::vector<ossimDpt3d>     theAttitudeSamples; // x=pitch, y=roll, z=yaw
   std::vector<ossimDpt3d>     theAttitudeVelSamples; // x=pitch/s, y=roll/s, z=yaw/s

   ossim_float64    getAttSampTime(int idx)const {return theAttSampTimes[idx];}
   //ossimDpt3d    getAttitude(int idx)const {return theAttitudeSamples[idx];}
   //ossimDpt3d    getAttitudeVel(int idx)const {return theAttitudeVelSamples[idx];}

   void updatePosEcfCoeff();
   void updateVelEcfCoeff();
protected:
   virtual ~radiZY3SupportData();

private:
	string time2String(const _TIMESTAMP& t)const;

	bool readHeadInfo(fstream& fs, QUIVIMAGE_HEAD_INFO& header);
	bool readTimeInfo(fstream& fs, _TIMESTAMP& t);
	bool readSAT_POS(fstream& fs, SAT_POS& pos);
	bool readSAT_ATT(fstream& fs, SAT_ATT& att);
	bool readQuivAuxInfo(fstream& fs, QUIVIMAGE_AUX_INFO& aux);
	void BitInverse(char* chrs, int nBit);

	template <class T>
	void BitInverse(T &data)
	{
		int nBit = sizeof(T);
		BitInverse((char*)&data, nBit);
	};

	void extrapolateTime(const ossim_float64& line, ossim_float64& time) const;
   void getLagrangeInterpolation(const ossim_float64& t,
                                 const std::vector<ossimDpt3d>& V,
                                 const std::vector<ossim_float64>& T,
                                 ossimDpt3d& li )const;

   void getBilinearInterpolation(const ossim_float64& t,
                                 const std::vector<ossimDpt3d>& V,
                                 const std::vector<ossim_float64>& T,
                                 ossimDpt3d& li )const;

   void getInterpolatedLookAngle(const ossim_float64& p,
                                 const std::vector<ossim_float64>& angles,
                                 ossim_float64& la) const;

   ossim_float64 convertTimeStamp(const ossimString& time_stamp) const;

   void convertTimeStamp(const ossimString& time_stamp,
                         ossim_float64& ti) const;

   /**
    * Initializes theMetadataVersion.
    * @return true on success, false if not found.
    */
   bool initMetadataVersion(ossimRefPtr<ossimXmlDocument> xmlDocument);

   /**
    * Initializes theImageId.
    * @return true on success, false if not found.
    */
   bool initImageId(ossimRefPtr<ossimXmlDocument> xmlDocument);

   /**
    * From xml section:
    * /Dimap_Document/Dataset_Sources/Source_Information/Scene_Source
    *
    * Initializes:
    * theSunAzimuth
    * theSunElevation
    * theIncidenceAngle
    * @return true on success, false if not found.
    */
   bool initSceneSource(ossimRefPtr<ossimXmlDocument> xmlDocument);

   /**
    * Frame points:
    *
    * From xml section:
    * /Dimap_Document/Dataset_Frame/
    *
    * Initializes:
    * theRefGroundPoint
    * theUlCorner
    * theUrCorner
    * theLrCorner
    * theLlCorner
    * theViewingAngle
    *
    * Note that the theRefImagePoint will be the zero based center of the
    * frame.
    * @return true on success, false if not found.
    */
   bool initFramePoints(ossimRefPtr<ossimXmlDocument> xmlDocument);

   // Extrapolates the attitude for imaging times outside the defined range:
   void extrapolateAttitude(const ossim_float64& time, ossimDpt3d& at) const;

   bool readCities(ossimFilename cityFile);
   bool readProvinces(ossimFilename provinceFile);
   bool readEph(ossimFilename ephFile);
   bool readNadir(ossimFilename nadirFile);


   ossimString                 theSensorID;
   ossimQVProcVersion		  theMetadataVersion;
   ossimFilename               theMetadataFile;
   ossim_uint32				  theProcessing;
   ossimString				  theStationID;
   ossimString				  theSpacecraftID;
   ossim_uint32				  theResampleRate;
   ossimString				  theStartTime;
   ossimString				  theStopTime;
   //ossim_uint32				  theColumns;
   //ossim_uint32				  theLines;
   ossim_uint32				  thePyramidLevels;
   ossim_uint32				  theImageTileSide;
   bool						  theCity;
   bool						  theProvince;
   bool						  theAuxInfo;
   bool						  theEph;
   bool						  theNadir;
   ossim_uint32				  theGridTileSide;
   ossim_float64              theReferenceTime;
   ossim_float64              theReferenceTimeLine;

   std::vector< std::vector<ossimDpt> > theProvinceList;
   std::vector<ossim_int32>  theEphSampLines;
   std::vector<ossimDpt>  theNadirList;

   ossimString                 theImageID;
   ossimString                 theProductionDate;
   ossimString                 theInstrument;
   ossim_uint32                theInstrumentIndex;

   /*
    * From xml section:
    * /Dimap_Document/Dataset_Sources/Source_Information/
    * Scene_Source
    */
   ossim_float64               theSunAzimuth;
   ossim_float64               theSunElevation;
   ossim_float64               theIncidenceAngle;
   ossim_float64               theViewingAngle;
   ossim_float64               theSceneOrientation;   
   
   ossimDpt                    theImageSize;

   /** Center of frame on ground, if sub image it's the center of that. */
   ossimGpt                    theRefGroundPoint;

   /** Zero based center of frame. */
   ossimDpt                    theRefImagePoint;

   ossimDpt                    theSubImageOffset;

   ossim_float64               theLineSamplingPeriod;

   /** holds the size of thePixelLookAngleX/Y */
   ossim_uint32                theDetectorCount;
   std::vector<ossim_float64>  thePixelLookAngleX;
   std::vector<ossim_float64>  thePixelLookAngleY;
   std::vector<ossim_float64>  theAttSampTimes;
   std::vector<ossimDpt3d>     thePosEcfSamples;
   std::vector<ossimDpt3d>     theVelEcfSamples;
   std::vector<ossim_float64>  theEphSampTimes;

   std::vector<int>            theLineSamples;
   std::vector<ossim_float64>  theLineSampTimes;
   bool                        theStarTrackerUsed;
   bool                        theSwirDataFlag;
   ossim_uint32                theNumBands;
   ossimString                 theAcquisitionDate;
   ossim_uint32                theStepCount;

   arma::vec thePosEcfCoeff;
   arma::vec theVelEcfCoeff;
   

   //---
   // Corner points:
   //---
   ossimGpt theUlCorner;
   ossimGpt theUrCorner;
   ossimGpt theLrCorner;
   ossimGpt theLlCorner;

   //---
   // Geoposition Points:
   //---
   std::vector <ossimDpt> theGeoPosImagePoints;
   std::vector <ossimGpt> theGeoPosGroundPoints;

   ossimGpt createGround(const ossimString& s)const;
   ossimDpt createDpt(const ossimString& s)const;

   /** callibration information for radiometric corrections*/

   std::vector<ossim_float64> thePhysicalBias;
   std::vector<ossim_float64> thePhysicalGain;

   std::vector<ossim_float64> theSolarIrradiance;
};
}
#endif /* #ifndef ossimSpotDimapSupportData_HEADER */
