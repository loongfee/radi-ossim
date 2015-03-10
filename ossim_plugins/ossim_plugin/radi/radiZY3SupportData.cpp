#include "radiZY3SupportData.h"
#include <ossim/base/ossimTrace.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimKeyword.h>
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/base/ossimDatum.h>
#include <ossim/base/ossimFilename.h>

#include "gdal_priv.h"
#include "cpl_string.h"

namespace ossimplugins{

	static ossimTrace traceDebug ("radiZY3SupportData:debug");

	static const ossim_uint32  LAGRANGE_FILTER_SIZE = 8; // num samples considered

	radiZY3SupportData::radiZY3SupportData ()
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
	radiZY3SupportData::radiZY3SupportData(const radiZY3SupportData& rhs)
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

	radiZY3SupportData::radiZY3SupportData (const ossimFilename& descFile, bool  processSwir)
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

		parseQVProcFile(descFile, processSwir);

		// Finished successful parse:
		if (traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
				<< "ossimQVProcSupportData::ossimQVProcSupportData: leaving..."
				<< std::endl;
		}
	}

	radiZY3SupportData::~radiZY3SupportData ()
	{
	}

	ossimObject* radiZY3SupportData::dup()const
	{
		return new radiZY3SupportData(*this);
	}

	void radiZY3SupportData::clearFields()
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


	bool radiZY3SupportData::parseQVProcFile(const ossimFilename& file,
		bool processSwir/*=false*/)
	{
		static const char MODULE[] = "radiZY3SupportData::parseQVProcFile";
		if(traceDebug())
		{
			ossimNotify(ossimNotifyLevel_DEBUG)
				<< MODULE << " DEBUG:"
				<< "\nFile: " << file << std::endl;
		}

		clearFields();
		theSwirDataFlag = processSwir;
		theMetadataFile = file;

		int last_pos = 0;
		GDALAllRegister();
		//OGRRegisterAll();//注册所有的文件格式驱动
		CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO"); // gdal 中文路径支持
		CPLSetConfigOption("SHAPE_ENCODING", "");	// shapefile 中文字段支持

		//ifstream fs(file.c_str(), std::ios_base::binary);
		fstream fs;
		fs.open(file.c_str(), std::ios_base::binary|std::ios_base::in);
		if(!fs)
		{
			cerr<<"open error!"<<endl;
			return false;
		}
		fs.seekg(0, ios::end);
		ossim_int64 file_size = fs.tellg();
		fs.seekg(0, ios::beg);

		cout<<"parsing orbit information..."<<endl;

		// 读QUIVIMAGE_HEAD_INFO
		QUIVIMAGE_HEAD_INFO theHeaderInfo;
		readHeadInfo(fs, theHeaderInfo);

		int line_offset = 0;

		theHeaderInfo.band_num = theHeaderInfo.gray_image_flag?1:3;
		int lineDataByte = theHeaderInfo.band_num * theHeaderInfo.data_width * theHeaderInfo.sample_bit_count/8;
		theHeaderInfo.line_num = (file_size-180)/(92+lineDataByte);
		theHeaderInfo.sample_num = theHeaderInfo.data_width;

		theProcessing = false;
		theStationID = theHeaderInfo.station_id;
		theSpacecraftID = theHeaderInfo.satellite_id;
		theSensorID = theHeaderInfo.sensor_id;
		theSensorID = theSpacecraftID + " " + theSensorID;
		theResampleRate = theHeaderInfo.sample_bit_count;
		theStartTime = time2String(theHeaderInfo.start_time).c_str();
		theStopTime = time2String(theHeaderInfo.end_time).c_str();
		theImageSize.samp = theHeaderInfo.sample_num;
		theImageSize.line = theHeaderInfo.line_num;
		theNumBands = theHeaderInfo.band_num;
		thePyramidLevels = theHeaderInfo.desample_num;


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

		int iLine = 0;
		int lastPercent = 0;
		while (!fs.eof()) {
			// 读取星历参数
			QUIVIMAGE_AUX_INFO aux;
			readQuivAuxInfo(fs, aux);
			if (0 == aux.valid_flag)
			{

				if (iLine == 0 || aux.line_count == 0)
				{
					aux.line_num = iLine - aux.line_count - line_offset;
					if (aux.satpos.x == 0 && aux.satpos.y == 0 && aux.satpos.y ==0)
					{
						//// 无效行
					}
					else
					{
						thePosEcfSamples.push_back(ossimDpt3d(aux.satpos.x, aux.satpos.y, aux.satpos.z));
						theVelEcfSamples.push_back(ossimDpt3d(aux.satpos.vx, aux.satpos.vy, aux.satpos.vz));
						theEphSampTimes.push_back(aux.gps_time);

						theAttitudeSamples.push_back(ossimDpt3d(aux.satatt.yaw, aux.satatt.pitch, aux.satatt.roll));
						theAttitudeBias.push_back(ossimDpt3d(0.0,0.0,0.0));
						theAttitudeVelSamples.push_back(ossimDpt3d(aux.satatt.vyaw, aux.satatt.vpitch, aux.satatt.vroll));
						theAttitudeVelBias.push_back(ossimDpt3d(0.0,0.0,0.0));
						theAttSampTimes.push_back(aux.att_time);

						theLineSamples.push_back(aux.line_num-line_offset);
						theLineSampTimes.push_back(aux.star_time);
					}
				}
			}

			if ((ossim_int64)fs.tellg() + lineDataByte > file_size)
			{
				break;
			}
			//fs.seekg(lineDataByte, ios_base::cur);
			fs.ignore(lineDataByte);
			//cout<<fs.tellg()<<endl;

			streampos pos_ = fs.tellg(); //读取文件指针的位置
			int currentPercent = (int)(pos_ / (double)(file_size)* 100 + 0.5);
			if (lastPercent != currentPercent)
			{
				printf("\r%d%%", currentPercent);
				fflush(stdout);
				lastPercent = currentPercent;
			}
			iLine++;
		}
		printf("\r%d%%", 100);
		printf("\n");

		// check


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


	string radiZY3SupportData::time2String(const _TIMESTAMP& t)const
	{
		const int bufSize = 2048;
		char buf[bufSize];
		sprintf_s(buf, bufSize, "%d-%d-%dT%d:%d:%d:%d", t.year, t.mon, t.day, t.hour, t.min, t.sec, t.nsec);
		return string(buf);
	}

	bool radiZY3SupportData::readHeadInfo(fstream& fs, QUIVIMAGE_HEAD_INFO& header)
	{
		//	接收该数据的地面站的标识 16
		fs.read((char*)header.station_id, sizeof(header.station_id));
		//	对应的卫星标识 16
		fs.read((char*)header.satellite_id, sizeof(header.satellite_id));
		//	对应的传感器标识 16
		fs.read((char*)header.sensor_id, sizeof(header.sensor_id));
		//	对应的工作模式信息 16
		fs.read((char*)header.work_mode, sizeof(header.work_mode));
		//	任务标识 20
		fs.read((char*)header.JobTaskID, sizeof(header.JobTaskID));
		//	轨道号 4
		fs.read((char*)&header.orbit_num, sizeof(header.orbit_num));
		//	通道号 4
		fs.read((char*)&header.channel_num, sizeof(header.channel_num));
		//	本任务单传输快视文件总数 4
		fs.read((char*)&header.totalfiles, sizeof(header.totalfiles));
		//	本次传输的文件在所属任务中的文件序号 4
		fs.read((char*)&header.files_num, sizeof(header.files_num));
		//	进行快数处理时的降采样数,上一步的降采样数 4
		fs.read((char*)&header.desample_num, sizeof(header.desample_num));
		//	上一步传进来的宽度,(如果我在做降采样的话),这里会进一步变小 4
		fs.read((char*)&header.data_width, sizeof(header.data_width));
		//	快视处理后的图像量化比特数，只有8Bit和16Bit两种情况 4
		fs.read((char*)&header.sample_bit_count, sizeof(header.sample_bit_count));
		//	当此值为真时，该图像为单通道灰度图像，否则为多通道伪彩色图像 4
		fs.read((char*)&header.gray_image_flag, sizeof(header.gray_image_flag));

		//	数据接收的开始时间 28
		readTimeInfo(fs, header.start_time);
		//  数据接收的结束时间 28 
		readTimeInfo(fs, header.end_time);

		//	对应的地面站在地球上的位置信息 8
		fs.read((char*)&header.station_pos.longitude, sizeof(header.station_pos.longitude));
		fs.read((char*)&header.station_pos.latitude, sizeof(header.station_pos.latitude));
		return true;
	}

	bool radiZY3SupportData::readTimeInfo(fstream& fs, _TIMESTAMP& t)
	{
		//	year 4
		fs.read((char*)&t.year, sizeof(int));
		//	mon 4
		fs.read((char*)&t.mon, sizeof(int));
		//	day 4
		fs.read((char*)&t.day, sizeof(int));
		//	hour 4
		fs.read((char*)&t.hour, sizeof(int));
		//	min 4
		fs.read((char*)&t.min, sizeof(int));
		//	sec 4
		fs.read((char*)&t.sec, sizeof(int));
		//	nsec 4
		fs.read((char*)&t.nsec, sizeof(int));
		return true;
	}

	bool radiZY3SupportData::readSAT_POS(fstream& fs, SAT_POS& pos)
	{
		//	x 4
		fs.read((char*)&pos.x, sizeof(float));
		//	y 4
		fs.read((char*)&pos.y, sizeof(float));
		//	z 4
		fs.read((char*)&pos.z, sizeof(float));
		//	vx 4
		fs.read((char*)&pos.vx, sizeof(float));
		//	vy 4
		fs.read((char*)&pos.vy, sizeof(float));
		//	vz 4
		fs.read((char*)&pos.vz, sizeof(float));
		return true;
	}

	bool radiZY3SupportData::readSAT_ATT(fstream& fs, SAT_ATT& att)
	{
		//	roll 4
		fs.read((char*)&att.roll, sizeof(float));
		//	pitch 4
		fs.read((char*)&att.pitch, sizeof(float));
		//	yaw 4
		fs.read((char*)&att.yaw, sizeof(float));
		const float pi = 3.14159265358979f;
		att.roll *= pi/180.0f;
		att.pitch *= pi/180.0f;
		att.yaw *= pi/180.0f;
		return true;
	}

	void radiZY3SupportData::BitInverse(char* chrs, int nBit)
	{
		char* tmp = new char[nBit];
		int result = 0;
		for (int i = 0;i < nBit;++i)
		{
			tmp[i] = chrs[nBit-1-i];
		}
		memcpy(chrs, tmp, nBit);
	}

	bool radiZY3SupportData::readQuivAuxInfo(fstream& fs, QUIVIMAGE_AUX_INFO& aux)
	{
		bool bState = true;
		//	该值为=1时有效，其他情况下辅助信息为无效值 4
		fs.read((char*)&aux.valid_flag, sizeof(aux.valid_flag));
		if (0 == aux.valid_flag)
		{
			// 辅助信息为无效值
			// 识别码 6
			char identifier[6];
			fs.read((char*)identifier, sizeof(identifier));
			// 路标识
			char pathId;
			fs.read((char*)&pathId, 1);

			// 星务对时码 秒 4
			unsigned int Star_s;
			//int Star_s;
			int nn = sizeof(Star_s);
			fs.read((char*)&Star_s, sizeof(Star_s));
			BitInverse((char*)&Star_s, sizeof(Star_s));
			// 星务对时码 毫秒 2
			unsigned short int Star_ms;
			fs.read((char*)&Star_ms, sizeof(Star_ms));
			BitInverse((char*)&Star_ms, sizeof(Star_ms));

			// 累计秒数 秒 4
			unsigned int GPS_s;
			fs.read((char*)&GPS_s, sizeof(GPS_s));
			BitInverse((char*)&GPS_s, sizeof(GPS_s));
			// 累计秒数 毫秒 2
			unsigned short int GPS_ms;
			fs.read((char*)&GPS_ms, sizeof(GPS_ms));
			BitInverse((char*)&GPS_ms, sizeof(GPS_ms));
			// X轴位置坐标 4字节补码（0.1m） 4
			int x;
			fs.read((char*)&x, sizeof(x));
			BitInverse((char*)&x, sizeof(x));
			// Y轴位置坐标 4字节补码（0.1m） 4
			int y;
			fs.read((char*)&y, sizeof(y));
			BitInverse((char*)&y, sizeof(y));
			// Z轴位置坐标 4字节补码（0.1m） 4
			int z;
			fs.read((char*)&z, sizeof(z));
			BitInverse((char*)&z, sizeof(z));
			// X轴速度坐标 4字节补码（0.01m/s） 4
			int vx;
			fs.read((char*)&vx, sizeof(vx));
			BitInverse((char*)&vx, sizeof(vx));
			// Y轴速度坐标 4字节补码（0.01m/s） 4
			int vy;
			fs.read((char*)&vy, sizeof(vy));
			BitInverse((char*)&vy, sizeof(vy));
			// Z轴速度坐标 4字节补码（0.01m/s） 4
			int vz;
			fs.read((char*)&vz, sizeof(vz));
			BitInverse((char*)&vz, sizeof(vz));

			// GPS对时码 秒 4
			unsigned int cGPS_s;
			fs.read((char*)&cGPS_s, sizeof(cGPS_s));
			BitInverse((char*)&cGPS_s, sizeof(cGPS_s));
			// GPS对时码 毫秒 2
			unsigned short int cGPS_ms;
			fs.read((char*)&cGPS_ms, sizeof(cGPS_ms));
			BitInverse((char*)&cGPS_ms, sizeof(cGPS_ms));

			// 姿控系统时间标准双精度型浮点数 秒 8
			double att_t;
			float d1, d2;
			fs.read((char*)&d1, 4);
			fs.read((char*)&d2, 4);
			BitInverse((char*)&d1, 4);
			BitInverse((char*)&d2, 4);
			memcpy((char*)&att_t+4, &d1, 4);
			memcpy((char*)&att_t, &d2, 4);
			//fs.read((char*)&att_t, sizeof(att_t));
			//BitInverse((char*)&att_t);
			// 滚动角（roll）标准浮点数 弧度 4
			float roll;
			fs.read((char*)&roll, sizeof(roll));
			BitInverse((char*)&roll, sizeof(roll));
			// 俯仰角（pitch）标准浮点数 弧度 4
			float pitch;
			fs.read((char*)&pitch, sizeof(pitch));
			BitInverse((char*)&pitch, sizeof(pitch));
			// 偏航角（yaw）标准浮点数 弧度 4
			float yaw;
			fs.read((char*)&yaw, sizeof(yaw));
			BitInverse((char*)&yaw, sizeof(yaw));
			// 滚动角速度（vroll）标准浮点数 弧度/秒 4
			float vroll;
			fs.read((char*)&vroll, sizeof(vroll));
			BitInverse((char*)&vroll, sizeof(vroll));
			// 俯仰角速度（pitch）标准浮点数 弧度/秒 4
			float vpitch;
			fs.read((char*)&vpitch, sizeof(vpitch));
			BitInverse((char*)&vpitch, sizeof(vpitch));
			// 偏航角速度（yaw）标准浮点数 弧度/秒 4
			float vyaw;
			fs.read((char*)&vyaw, sizeof(vyaw));
			BitInverse((char*)&vyaw, sizeof(vyaw));

			// CCD 行计数值 2
			//int nnn = sizeof(unsigned short);
			//unsigned short iLine;
			short line_count;
			//char c[2];
			//fs.read(c+1, 1);
			//fs.read(c+0, 1);
			//fs.read(c, 2);
			//memcpy((char*)&line_count, c, 2);
			//cout<<c[0]<<","<<c[1]<<endl;

			fs.read((char*)&line_count, sizeof(line_count));
			BitInverse((char*)&line_count, sizeof(line_count));

			// 暗像元 2
			unsigned short Dark_pixel;
			fs.read((char*)&Dark_pixel, sizeof(Dark_pixel));
			BitInverse((char*)&Dark_pixel, sizeof(Dark_pixel));

			// 后 3 字节填零
			fs.ignore(3);

			bState = false;

			//cout<<iLine<<endl;

			//////////////////////////////////////////////////////////////////////////
			aux.star_time = Star_s + Star_ms * 0.001;
			aux.gps_c_time = cGPS_s + cGPS_ms * 0.001;
			aux.gps_time = GPS_s + GPS_ms * 0.001;
			aux.satpos.x = x * 0.1f;
			aux.satpos.y = y * 0.1f;
			aux.satpos.z = z * 0.1f;
			aux.satpos.vx = vx * 0.01f;
			aux.satpos.vy = vy * 0.01f;
			aux.satpos.vz = vz * 0.01f;

			aux.satatt.roll = roll;
			aux.satatt.pitch = pitch;
			aux.satatt.yaw = yaw;
			aux.satatt.vroll = vroll;
			aux.satatt.vpitch = vpitch;
			aux.satatt.vyaw = vyaw;

			aux.att_time = att_t;
			aux.line_count = line_count;

			//cout<<line_count<<endl;
		}
		else if (1 == aux.valid_flag)
		{
			//	对应的星下点的位置 8
			fs.read((char*)&aux.nadir_pos.longitude, sizeof(aux.nadir_pos.longitude));
			fs.read((char*)&aux.nadir_pos.latitude, sizeof(aux.nadir_pos.latitude));
			//	该距离线对应的左侧的位置（精度和纬度） 8
			fs.read((char*)&aux.line_left_pos.longitude, sizeof(aux.line_left_pos.longitude));
			fs.read((char*)&aux.line_left_pos.latitude, sizeof(aux.line_left_pos.latitude));
			//	该距离线对应的右侧的位置（精度和纬度） 8
			fs.read((char*)&aux.line_right_pos.longitude, sizeof(aux.line_right_pos.longitude));
			fs.read((char*)&aux.line_right_pos.latitude, sizeof(aux.line_right_pos.latitude));
			//	卫星的x,y,z坐标以及vx,vy,vz 24
			readSAT_POS(fs, aux.satpos);
			//	卫星的姿态参数,俯仰滚动偏航，r,p,y 12
			readSAT_ATT(fs, aux.satatt);
			//  照射该脉冲的时间信息 28 
			readTimeInfo(fs, aux.line_time);
		}
		return bState;
	}


	bool radiZY3SupportData::saveState(ossimKeywordlist& kwl,
		const char* prefix)const
	{
		kwl.add(prefix,
			ossimKeywordNames::TYPE_KW,
			"radiZY3SupportData",
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

	bool radiZY3SupportData::loadState(const ossimKeywordlist& kwl,
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



	ossimGpt radiZY3SupportData::createGround(const ossimString& s)const
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

	ossimDpt radiZY3SupportData::createDpt(const ossimString& s)const
	{
		std::istringstream in(s.string());
		ossimString x, y;
		ossimString code;

		in >> x.string() >> y.string();

		return ossimDpt(x.toDouble(), y.toDouble());

	}


	void radiZY3SupportData::printInfo(ostream& os) const
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



	ossimString radiZY3SupportData::getSensorID() const
	{
		return theSensorID;
	}

	ossimString radiZY3SupportData::getSpacecraftID() const
	{
		return theSpacecraftID;
	}

	ossimString radiZY3SupportData::getStationID() const
	{
		return theStationID;
	}

	ossimString   radiZY3SupportData::getMetadataVersionString() const
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

	ossimString radiZY3SupportData::getAcquisitionDate() const
	{
		return theAcquisitionDate;
	}

	ossimString radiZY3SupportData::getProductionDate() const
	{
		return theProductionDate;
	}

	ossimString radiZY3SupportData::getImageID() const
	{
		return theImageID;
	}

	ossimFilename radiZY3SupportData::getMetadataFile() const
	{
		return theMetadataFile;
	}

	ossimString radiZY3SupportData::getInstrument() const
	{
		return theInstrument;
	}

	ossim_uint32 radiZY3SupportData::getInstrumentIndex() const
	{
		return theInstrumentIndex;
	}

	void radiZY3SupportData::getSunAzimuth(ossim_float64& az) const
	{
		az = theSunAzimuth;
	}

	void radiZY3SupportData::getSunElevation(ossim_float64& el) const
	{
		el = theSunElevation;
	}

	void radiZY3SupportData::getImageSize(ossimDpt& sz) const
	{
		sz = theImageSize;
	}

	void radiZY3SupportData::getLineSamplingPeriod(ossim_float64& pe) const
	{
		pe = theLineSamplingPeriod;
	}


	void radiZY3SupportData::getRefImagingTime(ossim_float64& t) const
	{
		t = theReferenceTime;
	}

	void radiZY3SupportData::getRefImagingTimeLine(ossim_float64& tl) const
	{
		tl = theReferenceTimeLine;
	}

	bool radiZY3SupportData::isStarTrackerUsed() const
	{
		return theStarTrackerUsed;
	}

	bool radiZY3SupportData::isSwirDataUsed() const
	{
		return theSwirDataFlag;
	}

	ossim_uint32 radiZY3SupportData::getNumberOfBands() const
	{
		return theNumBands;
	}

	ossim_uint32 radiZY3SupportData::getStepCount() const
	{
		return theStepCount;
	}

	void radiZY3SupportData::getIncidenceAngle(ossim_float64& ia) const
	{
		ia = theIncidenceAngle;
	}

	void radiZY3SupportData::getViewingAngle(ossim_float64& va) const
	{
		va = theViewingAngle;
	}

	void radiZY3SupportData::getSceneOrientation(ossim_float64& so) const
	{
		so = theSceneOrientation;
	}

	void radiZY3SupportData::getRefGroundPoint(ossimGpt& gp) const
	{
		gp = theRefGroundPoint;
	}

	void radiZY3SupportData::getRefImagePoint(ossimDpt& rp) const
	{
		rp = theRefImagePoint;
	}

	ossim_uint32 radiZY3SupportData::getNumSamples() const
	{
		return (ossim_uint32)theLineSamples.size();
	}

	ossim_uint32 radiZY3SupportData::getNumGeoPosPoints() const
	{
		return (ossim_uint32)theGeoPosImagePoints.size();
	}

	void radiZY3SupportData::getUlCorner(ossimGpt& pt) const
	{
		pt = theUlCorner;
	}

	void radiZY3SupportData::getUrCorner(ossimGpt& pt) const
	{
		pt = theUrCorner;
	}

	void radiZY3SupportData::getLrCorner(ossimGpt& pt) const
	{
		pt = theLrCorner;
	}

	void radiZY3SupportData::getLlCorner(ossimGpt& pt) const
	{
		pt = theLlCorner;
	}

	void radiZY3SupportData::getImageRect(ossimDrect& rect)const
	{
		rect = ossimDrect(0.0, 0.0, theImageSize.x - 1.0, theImageSize.y - 1.0);
	}

	void radiZY3SupportData::getSubImageOffset(ossimDpt& offset) const
	{
		offset = theSubImageOffset;
	}

}