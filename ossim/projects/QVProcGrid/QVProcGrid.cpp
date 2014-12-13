#include <stdlib.h>
#include <math.h>
#include <direct.h>


/////////////// gdal
#include "gdal_priv.h"
#include "gdal.h"
#include "ogr_srs_api.h"
#include "cpl_string.h"
#include "cpl_conv.h"
#include "cpl_multiproc.h"
#include "ogrsf_frmts.h"

#include <ossim_plugin/radi/radiQVProcSupportData.h>
#include <ossim_plugin/radi/ossimHj1Model.h>
#include <ossim/init/ossimInit.h>

#include <strUtil.h>
#include <fileUtil.h>
using namespace std;
using namespace mylib;


#pragma comment(lib, "ossim20.lib")
#pragma comment(lib, "mylib.lib")
#pragma comment(lib, "ossim_plugin.lib")
#pragma comment(lib, "mlpack.lib")
#pragma comment(lib, "OpenThreads.lib")
#pragma comment(lib, "gdal_i.lib")



#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include <OpenThreads/Barrier>

#ifdef _WIN32
#include <process.h>
#define getpid() _getpid()
#else
#include <unistd.h>
#endif 

const char* VERSION = "1.0.1";

static OpenThreads::Barrier bar;
static int GLOBAL_NUM_THREADS;
static int totalBlocks;
static int finishedBlocks;

const int percentList[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
static int fill_pos = 0;
static int percent = 0;
static float total_size;
int lastPercent = 0;
static GDALDatasetH hDstDS = new GDALDatasetH;
static ossimRefPtr<ossimplugins::ossimHj1Model> sensorModel;

static int mainThreadId;

const char *pszInputFile = "";
const char *pszOutFile = "";
int stepX = 32;
int stepY = 32;
int num_threads = 0;
int startLine = 0;
int endLine = 0;

class processThread: public OpenThreads::Thread
{
public:
	processThread()
		: OpenThreads::Thread(){};
	void setData(const ossimIrect& rect,
		int stepX = 32,
		int stepY = 32)
	{
		m_stepX = stepX;
		m_stepY = stepY;
		m_rect = rect;
	};
	virtual  ~processThread()
	{
		//m_pRegistration = NULL;
	};
	virtual void run()
	{
		bar.block(GLOBAL_NUM_THREADS);  // Sync the threads.

		int nCols = m_rect.width();
		int nRows = m_rect.height();
		int xOffset = m_rect.ul().x;
		int yOffset = m_rect.ul().y;
		int nBand = 2;
		vector<int> bandMap;
		GDALDataType eDT = GDALDataType::GDT_Float32;
		for (int i = 0;i < nBand;++i)
		{
			bandMap.push_back(i+1);
		}

		float *pData = new float [nCols * nRows * nBand];


		for (int i = 0;i < nRows;i += 1)
		{
			for (int j = 0;j < nCols;j += 1)
			{
				ossimGpt gpt;
				sensorModel->lineSampleToWorld(ossimDpt((xOffset + j)*stepX, startLine + (yOffset + i)*stepY), gpt);
				pData[0 * nCols * nRows + i * nCols + j] = gpt.lon;
				pData[1 * nCols * nRows + i * nCols + j] = gpt.lat;

				_quitmutex.lock();
				percent++;
				_quitmutex.unlock();

				//if (int(percent*100.0f/(float)total_size+0.5) == percentList[fill_pos])
				//{
				//	_quitmutex.lock();
				//	if (int(percent*100.0f / (float)total_size + 0.5) == percentList[fill_pos])
				//		cout<<" "<<percentList[fill_pos]<<"%";
				//	fill_pos++;
				//	_quitmutex.unlock();
				//}	

				//if (myId == mainThreadId)
				//{
				//	if (int(percent*100.0f/(float)total_size+0.5) == percentList[fill_pos])
				//	{
				//		cout<<" "<<percentList[fill_pos]<<"%";
				//		fill_pos++;
				//	}	
				//	//fflush( stdout );
				//}
				if (myId == mainThreadId)
				{
					int currentPercent = int(percent*100.0f / (float)total_size + 0.5);
					if (lastPercent != currentPercent)
					{
						printf("\r%d%%", currentPercent);
						fflush(stdout);
						lastPercent = currentPercent;
					}
				}
			}
		}

		_quitmutex.lock();
		((GDALDataset*)hDstDS)->RasterIO(GF_Write, 0, yOffset, nCols, nRows, pData, nCols, nRows, eDT, nBand,
			&bandMap[0], 0, 0, 0);
		_quitmutex.unlock();

		CPLFree( pData );

		bar.block(GLOBAL_NUM_THREADS);  // Sync the threads.

		//---------------------------------------------------------------------
		// Now that we've done our work, wait for a sign that we should quit.
		//
		while (true) {

			_quitmutex.lock();
			if(_quitflag == true) break;
			_quitmutex.unlock();

			OpenThreads::Thread::YieldCurrentThread();
		}
	};
	void quit() {
		_quitmutex.lock();
		_quitflag = true;
		_quitmutex.unlock();
	};

	void setId(const int &id)
	{
		myId = id;
	}
private:
	int myId;
	ossimIrect m_rect;
	int m_stepX;
	int m_stepY;
	int *_dataPtr;
	int _numElts;
	volatile bool _quitflag;
	OpenThreads::Mutex _quitmutex;
};

void createTiffGrid(ossimRefPtr<ossimplugins::radiQVProcSupportData> supportData)
{

	sensorModel = new ossimplugins::ossimHj1Model(supportData.get());

	//
	if (endLine <= 0)
	{
		// 如果未设置结束行，则默认为图像最后一行
		endLine = sensorModel->imageSize().y;
	}
	if (startLine > endLine)
	{
		cout << "起始行号不能大于结束行号." << endl;
		return;
	}

	int inSizeX = sensorModel->imageSize().x;
	int inSizeY = endLine - startLine + 1;

	int outSizeX = int(inSizeX / (double)stepX + 0.5) + 1;
	int outSizeY = int(inSizeY / (double)stepY + 0.5) + 1;


	int nHeight = outSizeY;
	int nWidth = outSizeX;
	total_size = nWidth * nHeight;


	GDALDriverH hDriver = GDALGetDriverByName("GTiff");
	GDALDataType eDT = GDALDataType::GDT_Float32;
	int dataTypeSize = GDALGetDataTypeSize(eDT) / 8;


	int nBand = 2;
	vector<int> bandMap;
	for (int i = 0; i < nBand; ++i)
	{
		bandMap.push_back(i + 1);
	}
	hDstDS = GDALCreate(hDriver, pszOutFile, nWidth, nHeight, nBand, eDT, NULL);


	if (num_threads == 0)
	{
		num_threads = OpenThreads::GetNumberOfProcessors() * 2;
	}
	//num_threads = 1;
	cout << "creating lon lat grid (using " << num_threads << " threads)" << "..." << endl;
	GLOBAL_NUM_THREADS = num_threads + 1;
	std::vector<processThread *> threads(num_threads);
	OpenThreads::Thread::SetConcurrency(num_threads);
	OpenThreads::Thread::Init();
	cout << percentList[fill_pos++] << "%";
	int TILE_HEIGHT = ceil(nHeight / (double)(num_threads));
	int BufWidth = nWidth;
	int BufHeight = TILE_HEIGHT;
	for (int i = 0; i < num_threads; ++i)
	{
		threads[i] = new processThread();
		//列末尾小块处理
		if (i == num_threads - 1)
		{
			BufHeight = nHeight - i * TILE_HEIGHT;
			BufHeight = min(BufHeight, TILE_HEIGHT);
		}

		ossimIrect rect(0, i*TILE_HEIGHT, nWidth - 1, i*TILE_HEIGHT + BufHeight - 1);
		threads[i]->setData(rect, stepX, stepY);
		threads[i]->setId(i);
	}

	mainThreadId = 0;
	lastPercent = 0;
	for (int i = 0; i < num_threads; ++i)
	{
		threads[i]->start();
	}

	bar.block(GLOBAL_NUM_THREADS);  // Block 'till ready
	bar.block(GLOBAL_NUM_THREADS);  // Block 'till finished

	//if (fill_pos < sizeof(percentList) / sizeof(percentList[0]))
	//{
	//	cout << " 100%";
	//}
	printf("\r%d%%", 100);
	//printf("\n");
	cout << endl;


	GDALClose(hDstDS);
}

void createTextGrid(ossimRefPtr<ossimplugins::radiQVProcSupportData> supportData)
{
	sensorModel = new ossimplugins::ossimHj1Model(supportData.get());

	//
	if (endLine <= 0)
	{
		// 如果未设置结束行，则默认为图像最后一行
		endLine = sensorModel->imageSize().y;
	}
	if (startLine > endLine)
	{
		cout << "起始行号不能大于结束行号." << endl;
		return;
	}

	int inSizeX = sensorModel->imageSize().x;
	int inSizeY = endLine - startLine + 1;

	int outSizeX = int(inSizeX / (double)stepX + 0.5) + 1;
	int outSizeY = int(inSizeY / (double)stepY + 0.5) + 1;
	total_size = outSizeX * outSizeY;

	cout << "creating lon lat grid (using " << 1 << " threads)" << "..." << endl;

	lastPercent = 0;
	FILE *pf = fopen(pszOutFile, "w+");
	for (int i = 0; i < outSizeY; i++)
	{
		if (i > 0)
		{
			fprintf(pf, "\n");
		}
		for (int j = 0; j < outSizeX; j++)
		{
			if (j > 0)
			{
				fprintf(pf, "\t");
			}
			ossimGpt gpt;
			sensorModel->lineSampleToWorld(ossimDpt(j*stepX, startLine + i*stepY), gpt);
			fprintf(pf, "%.6lf\t%.6lf\t%.6lf", gpt.lon, gpt.lat, 0.0);

			percent++;
			//if (int(percent*100.0f / (float)total_size + 0.5) == percentList[fill_pos])
			//{
			//	cout << " " << percentList[fill_pos] << "%";
			//	fill_pos++;
			//}
			int currentPercent = int(percent*100.0f / (float)total_size + 0.5);
			if (lastPercent != currentPercent)
			{
				printf("\r%d%%", currentPercent);
				fflush(stdout);
				lastPercent = currentPercent;
			}
		}
	}

	//if (fill_pos < sizeof(percentList) / sizeof(percentList[0]))
	//{
	//	cout << " 100%";
	//}
	printf("\r%d%%", 100);
	cout << endl;

	fclose(pf);
}

bool createLonLatGrid()
{
	ossimRefPtr<ossimplugins::radiQVProcSupportData> supportData = new ossimplugins::radiQVProcSupportData();
	if(!supportData->parseQVProcFile(pszInputFile))
	{
		return false;
	}

	// 检查数据格式
	ossimString sensorId = supportData->getSensorID();
	ossimString stationId = supportData->getStationID();
	if (!sensorId.upcase().contains("HJ"))
	{
		cout << "Only HJ satellite data are supported at present." << endl;
		return false;
	}
	//if (!stationId.upcase().contains("MYC"))
	//{
	//	cout << "Only the data from Miyun station are supported at present." << endl;
	//	return false;
	//}

	// 检查输出格式
	ossimFilename outputFile(pszOutFile);
	if (0 == strcmp(outputFile.ext().upcase(), "TIF")
		|| 0 == strcmp(outputFile.ext().upcase(), "TIFF"))
	{
		// TIFF 格式
		createTiffGrid(supportData);
	}
	else
	{
		// 否则输出文本格式
		createTextGrid(supportData);
	}

	return 1;
}

/*************************************************************************/
/*                               Usage()                                */
/************************************************************************/

static void Usage(const char* pszErrorMsg = NULL)

{
	printf( 
		"Usage: QVProcGrid -i inputFile -o outputFile(*.tif) [-stepX n] [-stepY n] [-sl startLine] [-el endLine] [-nt nthreads]\n"
		"  -stepX n: the step size in x direction, default value is 32.\n"
		"  -stepY n: the step size in y direction, default value is 32.\n"
		"  -sl startLine: start line number, default value is 0.\n"
		"  -el endLine: start line number, default value is 0 (stand for the max line of the image).\n"
		"  -nt nthreads: how many threads are used, default value is 0 (automatically assign according to the cores of processers, and only take effect if output file is tif format)\n"
		"\n"
		"QVProcGrid Version %s\n", VERSION);
	if( pszErrorMsg != NULL )
		fprintf(stderr, "\nFAILURE: %s\n", pszErrorMsg);
	exit(1);
}

#define CHECK_HAS_ENOUGH_ADDITIONAL_ARGS(nExtraArg) \
	do { if (i + nExtraArg >= argc) \
	Usage(CPLSPrintf("%s option requires %d argument(s)", argv[i], nExtraArg)); } while(0)

int main( int argc, char** argv )
{
	GDALAllRegister();
	//OGRRegisterAll();//注册所有的文件格式驱动
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO"); // gdal 中文路径支持
	CPLSetConfigOption("SHAPE_ENCODING", "");	// shapefile 中文字段支持

	int null_value = 0;


	if (argc > 1)
	{
		/* -------------------------------------------------------------------- */
		/*      Parse arguments.                                                */
		/* -------------------------------------------------------------------- */
		for( int i = 1; i < argc; i++ )
		{
			if( 0 == _stricmp(argv[i],"-i") )
			{
				CHECK_HAS_ENOUGH_ADDITIONAL_ARGS(1);
				pszInputFile = argv[++i] ;
			}
			else if( 0 == _stricmp(argv[i],"-o") )
			{
				CHECK_HAS_ENOUGH_ADDITIONAL_ARGS(1);
				pszOutFile = argv[++i] ;
			}
			else if( 0 == _stricmp(argv[i],"-stepX") )
			{
				CHECK_HAS_ENOUGH_ADDITIONAL_ARGS(1);
				stepX = atoi(argv[++i]) ;
			}
			else if( 0 == _stricmp(argv[i],"-stepY") )
			{
				CHECK_HAS_ENOUGH_ADDITIONAL_ARGS(1);
				stepY = atoi(argv[++i]) ;
			}
			else if (0 == _stricmp(argv[i], "-sl"))
			{
				CHECK_HAS_ENOUGH_ADDITIONAL_ARGS(1);
				startLine = atoi(argv[++i]);
			}
			else if (0 == _stricmp(argv[i], "-el"))
			{
				CHECK_HAS_ENOUGH_ADDITIONAL_ARGS(1);
				endLine = atoi(argv[++i]);
			}
			else if( 0 == _stricmp(argv[i],"-nt") )
			{
				CHECK_HAS_ENOUGH_ADDITIONAL_ARGS(1);
				num_threads = atoi(argv[++i]) ;
			}
			else
			{
				Usage();
			}
		}

		if (0 == strcmp(pszInputFile, ""))
		{
			printf("inputFile can not be empty!\n");
			Usage();
		}
		if (0 == strcmp(pszOutFile, ""))
		{
			printf("inputFile can not be empty!\n");
			Usage();
		}

		if (stepX < 1 || stepY < 1)
		{
			printf("stepX and stepY should greater than 1!\n");
			Usage();
		}

		else
		{
			ossimInit::instance()->initialize();
			clock_t  clockBegin, clockEnd;
			clockBegin = clock();
			createLonLatGrid();
			clockEnd = clock();
			printf("Time consuming: %lf s\n", (clockEnd - clockBegin)*1e-3);
		}
	}
	else
	{
		//pszInputFile = "E:\\HJ1\\HJ-1B_CCD-1\\HJ-1B_CCD-1_MYC_201404110131_201404110140\\data.dat";
		//pszInputFile = "E:\\HJ1\\HJ-1A_CCD-1\\HJ-1A_CCD-1_MYC_201403260030_201403260036.dat";
		//pszOutFile = "E:\\HJ1\\HJ-1A_CCD-1\\lonlat.tif";
		//ossimInit::instance()->initialize();
		//clock_t  clockBegin, clockEnd;
		//clockBegin = clock();
		//createLonLatGrid();
		//clockEnd = clock();
		//printf("Time consuming: %lf s\n", (clockEnd - clockBegin)*1e-3);
		Usage(0);
	}
	return 0;
}