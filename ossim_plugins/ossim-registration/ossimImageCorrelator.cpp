// ossimImageCorrelator.cpp
#include <iostream>

#include "ossimImageCorrelator.h"
#include <ossim/imaging/ossimImageSource.h>
#include <ossim/imaging/ossimImageHandler.h>
#include <ossim/imaging/ossimCastTileSourceFilter.h>
#include <ossim/imaging/ossimImageRenderer.h>
#include <ossim/imaging/ossimCacheTileSource.h>
#include <ossim/imaging/ossimImageChain.h>
#include <ossim/imaging/ossimBandSelector.h>
#include <ossim/imaging/ossimImageHandlerRegistry.h>
#include <ossim/projection/ossimMapProjection.h>
#include <ossim/projection/ossimProjectionFactoryRegistry.h>
#include <ossim/projection/ossimImageViewProjectionTransform.h>
#include <ossim/base/ossimTrace.h>
#include <ossim/base/ossimXmlDocument.h>
#include <ossim/base/ossimFilenameProperty.h>
#include <ossim/base/ossimStringProperty.h>

using namespace std;

static ossimTrace traceDebug("ossimImageCorrelator:debug");

static omp_lock_t lock;


//#pragma comment(lib, "opencv_calib3d248.lib")
//#pragma comment(lib, "opencv_contrib248.lib")
#pragma comment(lib, "opencv_core248.lib")
#pragma comment(lib, "opencv_features2d248.lib")
#pragma comment(lib, "opencv_flann248.lib")
//#pragma comment(lib, "opencv_gpu248.lib")
#pragma comment(lib, "opencv_highgui248.lib")
//#pragma comment(lib, "opencv_imgproc248.lib")
//#pragma comment(lib, "opencv_legacy248.lib")
//#pragma comment(lib, "opencv_ml248.lib")
#pragma comment(lib, "opencv_nonfree248.lib")
//#pragma comment(lib, "opencv_objdetect248.lib")
//#pragma comment(lib, "opencv_photo248.lib")
//#pragma comment(lib, "opencv_stitching248.lib")
//#pragma comment(lib, "opencv_ts248.lib")
//#pragma comment(lib, "opencv_video248.lib")
//#pragma comment(lib, "opencv_videostab248.lib")

RTTI_DEF2(ossimImageCorrelator, "ossimImageCorrelator", ossimOutputSource, ossimProcessInterface);

ossimImageCorrelator::ossimImageCorrelator()
 : ossimOutputSource(NULL, // owner
                        0,
                        0,
                        true,
                        true),
   ossimProcessInterface(),
   theMaster(),
   theSlave(),
   theMasterBand(0),
   theSlaveBand(0),
   theScaleRatio(1.0),
   theSlaveAccuracy(50.0),
   thePointNumber(25),
   theSiftNfeatures(0),
   theSiftNOctaveLayers(3),
   theSiftContrastThreshold(0.04),
   theSiftEdgeThreshold(10),
   theSiftSigma(1.6),
   theProjectionType("M"),
   theMasterPointProj("G"),
   theSlavePointProj("I"),
   theHasRun(false),
   handlerM(NULL),
   handlerS(NULL),
   theMasterBandSelector(NULL),
   theSlaveBandSelector(NULL),
   theTset(),
   theStoreFlag(false)
{
   //default output name : XML
   //1.add all OSSIM blocks
   // ingredients :
   // 2x ossimImageSource (for master & slave)
   // 2x ossimImagerRenderer
   // 1x ossimHarrisCorners
   // 2x ossimCastTileFilter (to get floating point
   // 1x ossimChipMatch
   // 1x ossimTieGenerator

   // don't create sources (factories will do it)
   caster.push_back(new ossimCastTileSourceFilter());
   caster.push_back(new ossimCastTileSourceFilter());   
#if OSSIM_HAS_MPI
   theFileStream = NULL;
#endif
}

ossimImageCorrelator::~ossimImageCorrelator()
{
   //TBC : delete handlers created by factory?
   
   if(caster.size())
   {
      caster[0]->disconnect();
      caster[1]->disconnect();
      caster.clear();
   }
   if(theMChain.valid())
   {
      theMChain->disconnect();
   }
   if(theSChain.valid())
   {
      theSChain->disconnect();
   }
   theMChain = 0;
   theSChain = 0;
}

ossimString ossimImageCorrelator::getRole() const
{
   ossimString role = "unknown";
   
   //use slave or master projection
   if (theProjectionType == "S")
   {
      role="slave";
   }
   else if (theProjectionType == "M")
   {
      role="master";
   }
   else
   {
      cerr<<"ossimImageCorrelator::getRole unknown output projection, need to supply it"<<endl;
   }

   return role;
}

ossimImageHandler*  ossimImageCorrelator::getProjectionHandler()
{
   //use slave or master projection
   ossimImageHandler* projHandler = 0;
   if (theProjectionType == "S")
   {
      projHandler = handlerS.get();
   }
   else if (theProjectionType == "M")
   {
	   if (!handlerM)
	   {
		   handlerM = ossimImageHandlerRegistry::instance()->open(theLastMaster);
		   if (!handlerM)
		   {
			   cerr<<"ossimImageCorrelator"<<"::execute can't create handler for slave image  "<< theSlave <<endl;
			   return false;
		   }
		   if(theMChain.valid())
		   {
			   theMChain->disconnect();
		   }
		   theMChain = new ossimImageChain;
		   theMChain->add(handlerM.get());
	   }
      projHandler = handlerM.get();
   }
   else
   {
      cerr<<"ossimImageCorrelator::getProjectionHandler cannot get handler for " << getRole() <<endl;
   }
   return projHandler;
}

ossimRefPtr<ossimImageGeometry> ossimImageCorrelator::getOutputImageGeometry()
{
   ossimRefPtr<ossimImageGeometry> geom = 0;
   ossimKeywordlist prjKwl;
   if (!theProjectionFile.empty())
   {
	   prjKwl.addFile(theProjectionFile);
	   geom->loadState(prjKwl);
   }
   else
   {
	   ossimImageHandler* projHandler = getProjectionHandler();
	   if(projHandler)
	   {
		   geom = projHandler->getImageGeometry();
	   }
   }
   return geom;
}

//getOutputProjection() - define output projection
// according to projType
ossimMapProjection* ossimImageCorrelator::getOutputProjection()
{
   ossimMapProjection* mop = 0;

   ossimRefPtr<ossimImageGeometry> geom = getOutputImageGeometry();
   if( geom.valid() )
   {
      if ( geom->getProjection() )
      {
         mop = PTR_CAST(ossimMapProjection, geom->getProjection());
         if( !mop )
         {
            ossimDpt mpp = geom->getMetersPerPixel();
            ossimProjection* outProjection =
               ossimProjectionFactoryRegistry::instance()->
               createProjection(ossimString("ossimEquDistCylProjection"));
            mop = PTR_CAST(ossimMapProjection, outProjection);
         }
         
         if(mop)
         {
            mop->setDatum(ossimDatumFactory::instance()->wgs84());
            mop->update();

            // apply user scale factor (resize)
            // then hopefully overviews can be used
            if ( (theScaleRatio != 1.0) && (theScaleRatio>0) )
            {
               cout << "applying scale ratio on " << getRole() <<
                  ": "<<theScaleRatio << endl; //TBR?

               mop->applyScale(ossimDpt(1.0/theScaleRatio,1.0/theScaleRatio),
                               false);
            }
         }
      }
      else
      {
         cerr << "ossimImageCorrelator::getOutputProjection cannot create projection from " << getRole() <<" geometry." <<endl;
      }
   }
   else
   {
      cerr << "ossimImageCorrelator::getOutputProjection cannot get "
           <<getRole() << " geometry." << endl;
   }

   return mop;
}

// buildRenerer() - builds renderer for an imageSource
// accounts for :
//  -scale factor
//  -required projection
bool ossimImageCorrelator::buildRenderer(
   ossimImageChain* chain,
   ossimMapProjection* outProjection, 
   ossimImageRenderer* renderer,
   const ossimFilterResampler::ossimFilterResamplerType& stype ) const
{
   if(chain)
   {
      chain->add(new ossimCacheTileSource);
      ossimRefPtr<ossimImageGeometry> geom = chain->getImageGeometry();
      if(geom.valid()&&geom->getProjection())
      {       
         ossimImageViewProjectionTransform* transform = new ossimImageViewProjectionTransform;
         transform->setImageGeometry(geom.get());
         transform->setViewGeometry(new ossimImageGeometry(0, outProjection));
         renderer->setImageViewTransform(transform);
         renderer->getResampler()->setFilterType(stype);
         chain->add(renderer);
         chain->add(new ossimCacheTileSource);
      }
      else
      {
         cerr<<"ossimImageCorrelator"<<"::buildRenderer cannot get projection from master/slave"<<endl;
         return false;
      }      
   }
   else
   {
      cerr<<"ossimImageCorrelator"<<"::buildRenderer NULL source"<<endl;
      return false;
   }
   return true;
}

struct row_col{
	double row_idx;
	double col_idx;
	row_col(double r, double c)
	{
		row_idx = r;
		col_idx = c;
	}
};


//bool RowColCompare(const row_col& rc1, const row_col& rc2)
//{
//	return (fabs(rc1.row_idx)+fabs(rc1.col_idx)) < (fabs(rc2.row_idx)+fabs(rc2.col_idx));
//}

static int row_col_step = 7;
// multiple keywords
bool RowColCompare(const row_col& rc1, const row_col& rc2)
{
	//return (fabs(rc1.row_idx)+fabs(rc1.col_idx)) < (fabs(rc2.row_idx)+fabs(rc2.col_idx));
	int fd1 = fabs(rc1.row_idx)+fabs(rc1.col_idx);
	int fd2 = fabs(rc2.row_idx)+fabs(rc2.col_idx);
	int d1 = (int)(fd1 + 0.5);
	int d2 = (int)(fd2 + 0.5);
	int m1 = d1%row_col_step;
	int m2 = d2 %row_col_step;
	if (m1 == m2)
	{
		return fd1 < fd2;
	}
	return m1 < m2;
}


bool kpt_compare(const cv::KeyPoint& d1, const cv::KeyPoint& d2)
{
	return d1.response > d2.response;
}

void normalization(const cv::Mat& inMat, cv::Mat& outMat)
{
	cv::Scalar mean_value;// = cv::mean(inMat);
	cv::Mat stdDevMat;
	cv::meanStdDev(inMat, mean_value, stdDevMat);
	cv::divide(inMat-mean_value, stdDevMat, outMat);
}

void findGoodMatches(vector< vector< DMatch >  > all_matches_2, vector< DMatch >& good_matches, float nndrRatio = 0.80f)
{
	good_matches.clear();
	//for (int i = 0; i < (int)matches.size(); i++)
	//{
	//	good_matches.push_back(matches[i][0]);
	//}
	good_matches.reserve(all_matches_2.size());

	for (size_t i = 0; i < all_matches_2.size(); ++i)
	{ 
		if (all_matches_2[i].size() < 2)
			continue;

		const DMatch &m1 = all_matches_2[i][0];
		const DMatch &m2 = all_matches_2[i][1];

		if(m1.distance <= nndrRatio * m2.distance)
			good_matches.push_back(m1);     
	}
}


// loong
ossimIpt ossimImageCorrelator::slave2master(ossimProjection* slaveProjection,
									ossimProjection* masterProjection,
									ossimIpt slaveDpt)
{
	ossimDpt masterDpt;
	ossimGpt gpt;
	slaveProjection->lineSampleToWorld(slaveDpt, gpt);
	masterProjection->worldToLineSample(gpt, masterDpt);
	return ossimIpt(masterDpt.x, masterDpt.y);
}

ossimDpt ossimImageCorrelator::slave2master(ossimProjection* slaveProjection,
									ossimProjection* masterProjection,
									ossimDpt slaveDpt)
{
	ossimDpt masterDpt;
	ossimGpt gpt;
	slaveProjection->lineSampleToWorld(slaveDpt, gpt);
	masterProjection->worldToLineSample(gpt, masterDpt);
	return masterDpt;
}

ossimDpt ossimImageCorrelator::slave2master(ossimDpt slaveDpt)
{
	ossimDpt masterDpt;
	ossimGpt gpt;
	theSlaveProjection->lineSampleToWorld(slaveDpt, gpt);
	theMasterProjection->worldToLineSample(gpt, masterDpt);
	return masterDpt;
}

ossimIpt ossimImageCorrelator::slave2master(ossimIpt slaveDpt)
{
	ossimDpt masterDpt;
	ossimGpt gpt;
	theSlaveProjection->lineSampleToWorld(slaveDpt, gpt);
	theMasterProjection->worldToLineSample(gpt, masterDpt);
	return ossimIpt(masterDpt.x, masterDpt.y);
}

bool ossimImageCorrelator::createTileMat(const ossimRefPtr<ossimCastTileSourceFilter>& cast, const ossimIrect& rect, cv::Mat& outMat, ossim_uint32 resLevel)
{
	//get Inputs
	ossimImageSource* imageSource = PTR_CAST(ossimImageSource, cast.get());
	if (!imageSource)
	{
		return false;
	}
	ossimRefPtr<ossimImageData> imageData = NULL;
	imageData = imageSource->getTile(rect, resLevel); //same resLevel?? TBC
	if(!imageData.valid() || imageData->getDataObjectStatus() == OSSIM_EMPTY
		|| imageData->getDataObjectStatus() == OSSIM_PARTIAL)
	{
		imageSource = NULL;
		return false;
	}

	//outMat = cv::Mat(cv::Size(imageData->getWidth(), imageData->getHeight()), CV_64FC1);
	//outMat.data = static_cast<uchar*>(imageData->getBuf(0));
	//outMat.convertTo(outMat, CV_8UC1);
	outMat = cv::Mat(cv::Size(rect.width(), rect.height()), CV_8UC1);
	outMat.data = static_cast<uchar*>(imageData->getBuf(0));
	//memcpy(outMat.data, static_cast<uchar*>(imageData->getBuf(0)), imageData->getWidth()*imageData->getHeight());
	//imageData->unloadTile((void*)outMat.data, rect, ossimInterleaveType::OSSIM_INTERLEAVE_UNKNOWN);
	imageSource = NULL;
	return true;
}

bool ossimImageCorrelator::getMasterList(ossimFilename spatial_index_file, vector<ossimFilename>& masterList,
				   ossimGpt ul, ossimGpt lr)
{
	masterList.clear();
	GDALAllRegister();
	OGRRegisterAll();//注册所有的文件格式驱动

	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");

	OGRSFDriver* poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName("ESRI Shapefile");//得到shp文件的处理器

	OGRDataSource* poDS = poDriver->Open(spatial_index_file.c_str(), NULL);//打开文件
	
	OGRLayer* poLayer = poDS->GetLayer(0);//获取shp图层

	poLayer->SetSpatialFilterRect(ul.lon, ul.lat, lr.lon, lr.lat);
	//poLayer->SetSpatialFilterRect(ul.lat, ul.lon, lr.lat, lr.lon);

	OGRFeature *feature;
	while (feature = poLayer->GetNextFeature())
	{
		masterList.push_back(feature->GetFieldAsString("path"));
	}
	return true;
}
bool 
	ossimImageCorrelator::execute()
{
	bool result=true;
	if(theSChain.valid())
	{
		theSChain->disconnect();
	}
	theSChain = new ossimImageChain;

	handlerS = ossimImageHandlerRegistry::instance()->open(theSlave);
	if (!handlerS)
	{
		cerr<<"ossimImageCorrelator"<<"::execute can't create handler for slave image  "<< theSlave <<endl;
		return false;
	}
	theSChain->add(handlerS.get());

	ossim_uint32 sbc = handlerS->getNumberOfOutputBands();
	//add a band selector
	ossim_uint32 sb = getSlaveBand();
	if (sb>=sbc) 
	{
		cerr<<"ossimImageCorrelator"<<"::execute Warning not enough bands in slave, only "<< sbc <<endl;
		sb=0;
	}
#if OSSIM_HAS_MPI
	if (ossimMpi::instance()->getRank() == 0)
#endif
	{
		cout<<"Using band "<<sb<<" for slave"<<endl; //TBR
	}
	theSlaveBandSelector = new ossimBandSelector;
	theSlaveBandSelector->connectMyInputTo(0, handlerS.get());
	theSlaveBandSelector->setOutputBandList(vector<ossim_uint32>(1,sb));
	//      theSlaveSource = theSlaveBandSelector;
	theSChain->add(theSlaveBandSelector.get());


	//init casters
	//caster[0]->setOutputScalarType(OSSIM_FLOAT64);
	//caster[1]->setOutputScalarType(OSSIM_FLOAT64);
	caster[0]->setOutputScalarType(OSSIM_UCHAR);
	caster[1]->setOutputScalarType(OSSIM_UCHAR);

	//init gen
	setStoreFlag(true); //also keep all tie points in memory

	//TBD : set area of interest to buffer around slave?

	// -- 3 -- tie blocks, from sources to outputs

	caster[0]->connectMyInputTo(0, theSlaveBandSelector.get());
	//////////////////////////////////////////////////////////////////////////
	//ossimKeywordlist kwl;
	//handlerS->getImageGeometry()->getProjection()->saveState(kwl);
	//theSlaveProjection = ossimProjectionFactoryRegistry::instance()->createProjection(kwl);

	theSlaveProjection = handlerS->getImageGeometry()->getProjection();
	//////////////////////////////////////////////////////////////////////////

	theAreaOfInterest = handlerS->getBoundingRect(0);

	//open();

	theTset.clearTiePoints();
	// -- 4 -- run
	result = getAllFeatures();

#if OSSIM_HAS_MPI
	int myid = ossimMpi::instance()->getRank();
	char buf[1024];
	sprintf_s(buf, "%s_%d.%s",
		theFilename.fileNoExtension(),
		myid, theFilename.ext());
	ossimFilename tempFilename =  ossimFilename(buf);
	fstream ofs;
	ofs.open(tempFilename.c_str(), ios_base::out);
	ossimMapProjection* outProjection = ossimImageCorrelator::getOutputProjection();
	vector< ossimRefPtr<ossimTieGpt> >::const_iterator it;
	int icount = 0;
	for (it = theTset.getTiePoints().begin();it!=theTset.getTiePoints().end();++it)
	{
		char buf[1024];
		double hgt = ossimElevManager::instance()->getHeightAboveEllipsoid((*it)->getGroundPoint());
		ossimDpt dpt = (*it)->getImagePoint();
		ossimGpt gpt = (*it)->getGroundPoint();
		if (!outProjection->isGeographic())
		{
			ossimDpt d1 = outProjection->forward(gpt);
			gpt = ossimGpt(d1.x, d1.y, hgt);
			sprintf_s(buf, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\n\0", 
				icount+=1,
				dpt.x, 
				dpt.y,
				gpt.lat,
				gpt.lon,
				hgt);
			int nLength = strlen(buf);
			ofs<<buf;
		}
		else
		{
			sprintf_s(buf, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\n\0", 
				icount+=1,
				dpt.x, 
				dpt.y,
				gpt.lon,
				gpt.lat,
				hgt);
			int nLength = strlen(buf);
			ofs<<buf;
		}
	}
	ofs.close();
	MPI_Barrier(MPI_COMM_WORLD);
	if (ossimMpi::instance()->getRank() == 0)
	{
		fstream outFs;
		outFs.open(theFilename.c_str(), ios_base::out);
		for (int i = 0;i < ossimMpi::instance()->getNumberOfProcessors();++i)
		{
			char buf_[1024];
			sprintf_s(buf_, "%s_%d.%s",
			theFilename.fileNoExtension(),
			i, theFilename.ext());
			ossimFilename tmpfile =  ossimFilename(buf_);
			fstream inFs;
			inFs.open(tmpfile.c_str(), ios_base::in);
			char charBuf[1024];
			while (inFs.getline(charBuf, 1024))
			{
				outFs<<charBuf<<endl;
			}
			inFs.close();
			DeleteFileA(tmpfile);
		}
		outFs.close();

		fstream geomFs;
		geomFs.open(theFilename.setExtension("geom").c_str(), ios_base::out);
		ossimKeywordlist prjKwl;
		getOutputProjection()->saveState(prjKwl);
		geomFs<<prjKwl;
		geomFs.close();
	}
#else
	fstream geomFs;
	geomFs.open(theFilename.setExtension("geom").c_str(), ios_base::out);
	ossimKeywordlist prjKwl;
	getOutputProjection()->saveState(prjKwl);
	geomFs<<prjKwl;
	geomFs.close();
#endif

	//close();

	theHasRun = true;
	return true;
}

int ossimImageCorrelator::runMatchParallel(const cv::Mat& slaveMat, const cv::Mat& masterMat, ossimTDpt& tDpt, void* pData, bool bDebug)
{
	ossimImageCorrelator* pThis = (ossimImageCorrelator*)pData;
	// detect corners
	cv::initModule_nonfree();
	std::vector<KeyPoint> skeypoints, mkeypoints;
	Mat sdescriptors, mdescriptors;
	SIFT detector(pThis->theSiftNfeatures, pThis->theSiftNOctaveLayers, 
		pThis->theSiftContrastThreshold, pThis->theSiftEdgeThreshold, pThis->theSiftSigma);

	if (bDebug)
	{
		cv::imwrite("slave.png", slaveMat);
		cv::imwrite("master.png", masterMat);
	}

	// detect
	detector.detect( slaveMat, skeypoints );
	if (skeypoints.size() < 10 )
	{
		return match_state::slave_faild;

	}
	// detect
	detector.detect( masterMat, mkeypoints );
	if(mkeypoints.size() < 10)
	{
		//handlerM->close();
		return match_state::master_faild;
	}

	// extract
	cv::SiftDescriptorExtractor extractor;
	extractor.compute( slaveMat, skeypoints, sdescriptors );
	extractor.compute( masterMat, mkeypoints, mdescriptors );


	BFMatcher matcher(NORM_L1, false); 
	vector< vector< DMatch >  > matches;
	matcher.knnMatch( sdescriptors, mdescriptors, matches, 2 );
	
	vector< DMatch > good_matches;
	findGoodMatches(matches, good_matches, 0.70f);

	if (good_matches.size() < 4)
	{
		//handlerM->close();
		return match_state::match_failed;
	}

	//-- Create input data
	Eigen::MatrixXd dataPoints((int)good_matches.size(), 4);
	for(unsigned int i = 0;i < good_matches.size();++i)
	{
		dataPoints(i, 0) = skeypoints[good_matches[i].queryIdx].pt.x;
		dataPoints(i, 1) = skeypoints[good_matches[i].queryIdx].pt.y;
		dataPoints(i, 2) = mkeypoints[good_matches[i].trainIdx].pt.x;
		dataPoints(i, 3) = mkeypoints[good_matches[i].trainIdx].pt.y;
	}

	// RANSAC detect outliers
	auto_ptr< estimators::Solver<Eigen::MatrixXd, Eigen::VectorXd> > ptrSolver(
		new estimators::affineSolver<Eigen::MatrixXd,Eigen::VectorXd>);
	vector<int> inliers;
	//for (int i = 0; i < (int)good_matches.size(); i++) inliers.push_back(i);
	vector<Eigen::VectorXd> models;

	ransac::Ransac_Handler ransac_fun_Handler;
	bool result = ransac::Ransac_RobustEstimator
		(
		dataPoints, // the input data
		estimators::affineSolver<Eigen::MatrixXd, Eigen::VectorXd>::extractor, // How select sampled point from indices
		dataPoints.rows(),  // the number of putatives data
		*(ptrSolver.get()),  // compute the underlying model given a sample set
		estimators::affineSolver<Eigen::MatrixXd, Eigen::VectorXd>::defaultEvaluator,  // the function to evaluate a given model
		//Ransac Object that contain function:
		// CandidatesSelector, Sampler and TerminationFunction
		ransac_fun_Handler, // the basic ransac object
		1000,  // the maximum rounds for RANSAC routine
		inliers, // inliers to the final solution
		models, // models array that fit input data
		0.95 // the confidence want to achieve at the end
		);
	if (inliers.size() < 6)
	{
		//handlerM->close();
		return match_state::match_failed;
	}

	//if (fabs(models[0][1] * models[0][5] - models[0][2] * models[0][4]) < 0.5)
	//{
	//	//handlerM->close();
	//	return match_state::match_failed;
	//}
	if (fabs(1.0 - fabs(models[0][1] * models[0][5] - models[0][2] * models[0][4])) > 0.5)
	{
		//handlerM->close();
		return match_state::match_failed;
	}

	// maybe a lot of correspondences are found, but we need only one correspondence for a tile
	ossimIpt mc = ossimIpt(mkeypoints[good_matches[inliers[0]].trainIdx].pt.x, mkeypoints[good_matches[inliers[0]].trainIdx].pt.y);
	ossimIpt sc = ossimIpt(skeypoints[good_matches[inliers[0]].queryIdx].pt.x, skeypoints[good_matches[inliers[0]].queryIdx].pt.y);

	tDpt = ossimTDpt( mc, sc, good_matches[inliers[0]].distance );


	if (bDebug)
	{
		//cout<<models[0]<<endl;
		vector< DMatch > final_matches;
		for (int i = 0;i < (int)inliers.size();++i)
		{
			final_matches.push_back(good_matches[inliers[i]]);
		}
		// Draw matches
		cv::Mat imgMatch;
		drawMatches(slaveMat, skeypoints, masterMat, mkeypoints, final_matches, imgMatch);
		cv::imwrite("result.png", imgMatch);
	}

	//handlerM->close();
	return match_state::success;
}

int ossimImageCorrelator::runMatch(const ossimIrect &srect, const ossimIrect &mrect, vector<ossimTDpt>& theTies, ossim_uint32 resLevel)
{
	//erase stored tie points
	theTies.clear();

	//get Inputs
	ossimImageSource* slave = PTR_CAST(ossimImageSource, caster[0].get());
	ossimImageSource* master  = PTR_CAST(ossimImageSource, caster[1].get());
	if (!master)
	{
		return match_state::master_faild;
	}
	if( !slave)
	{
		//handlerM->close();
		return match_state::slave_faild;
	}

	long w = srect.width();
	long h = mrect.height();

	ossimIpt delta_lr((ossim_int32)(ceil(theSlaveAccuracy)), (ossim_int32)(ceil(theSlaveAccuracy)) );

	ossimRefPtr<ossimImageData> slaveData = slave->getTile(srect, resLevel); //same resLevel?? TBC
	if(!slaveData.valid() || slaveData->getDataObjectStatus() == OSSIM_EMPTY
		|| slaveData->getDataObjectStatus() == OSSIM_PARTIAL
		|| !isSourceEnabled())
	{
		//handlerM->close();
		return match_state::slave_faild;
	}

	// detect corners
	cv::initModule_nonfree();
	std::vector<KeyPoint> skeypoints, mkeypoints;
	Mat sdescriptors, mdescriptors;
	SIFT detector(theSiftNfeatures, theSiftNOctaveLayers, 
		theSiftContrastThreshold, theSiftEdgeThreshold, theSiftSigma);

	cv::Mat slaveMat(cv::Size(slaveData->getWidth(), slaveData->getHeight()), CV_64FC1);
	slaveMat.data = static_cast<uchar*>(slaveData->getBuf(0));
	slaveMat.convertTo(slaveMat, CV_8UC1);
	cv::imwrite("slave.png", slaveMat);
	// detect
	detector.detect( slaveMat, skeypoints );
	if (skeypoints.size() < 10 )
	{
		return match_state::slave_faild;

	}

	// master
	ossimRefPtr<ossimImageData> masterData = master->getTile(mrect, resLevel); //same resLevel?? TBC
	if(!masterData.valid() || masterData->getDataObjectStatus() == OSSIM_EMPTY
		|| masterData->getDataObjectStatus() == OSSIM_PARTIAL
		|| !isSourceEnabled())
	{
		//handlerM->close();
		return match_state::master_faild;
	}
	cv::Mat masterMat(cv::Size(masterData->getWidth(), masterData->getHeight()), CV_64FC1);
	masterMat.data = static_cast<uchar*>(masterData->getBuf(0));
	masterMat.convertTo(masterMat, CV_8UC1);
	cv::imwrite("master.png", masterMat);

	// detect
	detector.detect( masterMat, mkeypoints );
	if(mkeypoints.size() < 10)
	{
		//handlerM->close();
		return match_state::master_faild;
	}

	// extract
	cv::SiftDescriptorExtractor extractor;
	extractor.compute( slaveMat, skeypoints, sdescriptors );
	extractor.compute( masterMat, mkeypoints, mdescriptors );


	BFMatcher matcher(NORM_L1, false); 
	vector< vector< DMatch >  > matches;
	matcher.knnMatch( sdescriptors, mdescriptors, matches, 2 );
	
	vector< DMatch > good_matches;
	findGoodMatches(matches, good_matches, 0.80f);

	if (good_matches.size() < 4)
	{
		//handlerM->close();
		return match_state::match_failed;
	}

	//-- Create input data
	Eigen::MatrixXd dataPoints((int)good_matches.size(), 4);
	for(unsigned int i = 0;i < good_matches.size();++i)
	{
		dataPoints(i, 0) = skeypoints[good_matches[i].queryIdx].pt.x;
		dataPoints(i, 1) = skeypoints[good_matches[i].queryIdx].pt.y;
		dataPoints(i, 2) = mkeypoints[good_matches[i].trainIdx].pt.x;
		dataPoints(i, 3) = mkeypoints[good_matches[i].trainIdx].pt.y;
	}

	// RANSAC detect outliers
	auto_ptr< estimators::Solver<Eigen::MatrixXd, Eigen::VectorXd> > ptrSolver(
		new estimators::affineSolver<Eigen::MatrixXd,Eigen::VectorXd>);
	vector<int> inliers;
	//for (int i = 0; i < (int)good_matches.size(); i++) inliers.push_back(i);
	vector<Eigen::VectorXd> models;

	ransac::Ransac_Handler ransac_fun_Handler;
	bool result = ransac::Ransac_RobustEstimator
		(
		dataPoints, // the input data
		estimators::affineSolver<Eigen::MatrixXd, Eigen::VectorXd>::extractor, // How select sampled point from indices
		dataPoints.rows(),  // the number of putatives data
		*(ptrSolver.get()),  // compute the underlying model given a sample set
		estimators::affineSolver<Eigen::MatrixXd, Eigen::VectorXd>::defaultEvaluator,  // the function to evaluate a given model
		//Ransac Object that contain function:
		// CandidatesSelector, Sampler and TerminationFunction
		ransac_fun_Handler, // the basic ransac object
		1000,  // the maximum rounds for RANSAC routine
		inliers, // inliers to the final solution
		models, // models array that fit input data
		0.95 // the confidence want to achieve at the end
		);
	if (inliers.size() < 6)
	{
		//handlerM->close();
		return match_state::match_failed;
	}

	if (fabs(models[0][1] * models[0][5] - models[0][2] * models[0][4]) < 1e-2)
	{
		//handlerM->close();
		return match_state::match_failed;
	}

	vector< DMatch > final_matches;
	for (int i = 0;i < (int)inliers.size();++i)
	{
		final_matches.push_back(good_matches[inliers[i]]);
	}

	// maybe a lot of correspondences are found, but we need only one correspondence for a tile
	ossimIpt mc = ossimIpt(mkeypoints[final_matches[0].trainIdx].pt.x, mkeypoints[final_matches[0].trainIdx].pt.y) + mrect.ul();
	ossimIpt sc = ossimIpt(skeypoints[final_matches[0].queryIdx].pt.x, skeypoints[final_matches[0].queryIdx].pt.y) + srect.ul();
	theTies.push_back(ossimTDpt( mc, sc, final_matches[0].distance ));

	// Draw matches
	Mat imgMatch;
	drawMatches(slaveMat, skeypoints, masterMat, mkeypoints, final_matches, imgMatch);
	cv::imwrite("result.png", imgMatch);

	//handlerM->close();
	return match_state::success;
}
//
//bool ossimImageCorrelator::getGridFeatures(const ossimIrect& rect)
//{
//	if (!theSlaveBandSelector)
//	{
//		ossimNotify(ossimNotifyLevel_WARN)
//			<< "WARN ossimTieGenerator::scanForEdges():"
//			<< "\nInput source is not a ossimImageChip.  Returning..." << std::endl;
//		return false;
//	}
//
//	//const ossim_int32 TILE_HEIGHT    = src->getTileHeight();
//	//const ossim_int32 TILE_WIDTH     = src->getTileWidth();
//	const ossim_int32 TILE_HEIGHT	= theTileSize;
//	const ossim_int32 TILE_WIDTH	= theTileSize;
//	const ossim_int32 START_LINE = rect.ul().y;
//	const ossim_int32 STOP_LINE  = rect.lr().y;
//	const ossim_int32 START_SAMP = rect.ul().x;
//	const ossim_int32 STOP_SAMP  = rect.lr().x;
//	int nWidth = STOP_SAMP-START_SAMP;
//	int nHeight = STOP_LINE-START_LINE;
//
//	// For percent complete status.
//	ossim_int32 tilerows=(STOP_LINE-START_LINE+TILE_HEIGHT) / TILE_HEIGHT; //ceil : (stop-start+1+size-1)/size
//	ossim_int32 tilecols=(STOP_SAMP-START_SAMP+TILE_WIDTH) / TILE_WIDTH;
//	double total_tiles = ((double)tilerows)*tilecols;
//	double tiles_processed = 0.0;
//	
//	// loop through all tiles
//	// need to use a sequencer for parallelism in the future TBD
//	ossim_int32 line=START_LINE;
//	ossim_int32 i,j;
//
//	vector<row_col> row_col_List;
//	double center_row = tilerows * 0.5;
//	double center_col = tilecols * 0.5;
//	for (i=0;(i<tilerows);++i)
//	{
//		for (j=0;(j<tilecols);++j )
//		{
//			row_col_List.push_back(row_col((center_row-i-1),(center_col-j-1)));
//		}
//	}
//
//	std::sort(row_col_List.begin(), row_col_List.end(), RowColCompare);
//	for (int i=0;i < (int)row_col_List.size();++i)
//	{
//		int icol = floor(row_col_List[i].col_idx+center_col+0.5);
//		int irow = floor(row_col_List[i].row_idx+center_row+0.5);
//		ossim_int32 samp=START_SAMP+icol*TILE_WIDTH;
//		ossim_int32 line=START_LINE+irow*TILE_HEIGHT;
//
//
//		ossim_int32 BufHeight = TILE_HEIGHT;
//		ossim_int32 BufWidth = TILE_WIDTH;
//		//行末尾小块处理
//		if (irow == tilerows-1)
//		{
//			BufHeight = nHeight - (tilerows-1) * TILE_HEIGHT;//得出当前块的宽度Bufsizex，高度Bufsizey
//			BufHeight = min(BufHeight, TILE_HEIGHT);
//		}
//		//列末尾小块处理
//		if (icol == tilecols-1)
//		{
//			BufWidth = nWidth - (tilecols-1) * TILE_WIDTH;//得出当前块的宽度Bufsizex，高度Bufsizey
//			BufWidth = min(BufWidth, TILE_WIDTH);
//		}
//
//		vector<ossimTDpt> tp;
//		int match_result;
//
//		// slave
//		ossimIrect srect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1));
//		ossimIpt sul = srect.ul();
//		ossimIpt slr = srect.lr();
//		// master
//		ossimIpt delta_lr((ossim_int32)(ceil(theSlaveAccuracy)), (ossim_int32)(ceil(theSlaveAccuracy)) );
//		if (handlerM != NULL)
//		{
//			ossimIpt mul = slave2master(sul - delta_lr);
//			ossimIpt mlr = slave2master(slr + delta_lr);
//			ossimIrect mrect = ossimIrect(mul, mlr);
//			if(handlerM->getBoundingRect(0).pointWithin(mul) 
//				&& handlerM->getBoundingRect(0).pointWithin(mlr))
//			{
//				match_result = ossimImageCorrelator::runMatch(srect, mrect, tp);
//				if (match_result == match_state::slave_faild)
//				{
//					continue;
//				}
//				if (thedTieptFilename != ossimFilename::NIL && tp.size() > 0)
//				{
//					//write on stream
//					writeTiePoints(tp);
//				}
//				if (getStoreFlag())
//				{
//					//store them : insert at the end (constant time) //TBD : conditional store
//					for (int iPoint = 0;iPoint < (int)tp.size();++iPoint)
//					{
//						theTiePoints.push_back(tp[iPoint]);
//						// convert "Image to Image" to "Ground to Image" tie points    //TBC : use more generic tie points
//
//						ossimRefPtr<ossimTieGpt> tgi(new ossimTieGpt);
//						//set master ground pos
//						theMasterProjection->lineSampleToWorld( tp[iPoint].getMasterPoint() , *tgi ); //TBC : is it always lon/lat WGS84?
//						double hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(*tgi);
//						//ossimDpt dpt = outProj->forward(*tgi);
//						tgi->setGroundPoint(ossimGpt(tgi->lon, tgi->lat, hgt));
//						//set slave image position
//						tgi->refImagePoint() = tp[iPoint].getSlavePoint();
//						//set score
//						tgi->setScore(tp[iPoint].score);
//
//						//add to list
//						theTset.addTiePoint(tgi);
//					}
//					//theTiePoints.insert(theTiePoints.end(),tp.begin(),tp.end());
//				}
//				if (tp.size() > 0)
//				{
//					return true;
//				}
//			}
//		}
//		// search in the reference library
//		ossimGpt ul_latlon, lr_latlon;
//		theSlaveProjection->lineSampleToWorld(sul-delta_lr, ul_latlon);
//		theSlaveProjection->lineSampleToWorld(slr+delta_lr, lr_latlon);
//		vector<ossimFilename> masterFileList;
//		if (theMaster.ext().upcase() == "SHP")
//		{
//			getMasterList(theMaster, masterFileList, ul_latlon, lr_latlon);
//		}
//		else{
//			masterFileList.clear();
//			masterFileList.push_back(theMaster);
//		}
//		for (int iFile = 0;iFile < (int)masterFileList.size();++iFile)
//		{
//			theLastMaster = masterFileList[iFile];
//
//			if(theMChain.valid())
//			{
//				theMChain->disconnect();
//			}
//			theMChain = new ossimImageChain;
//			handlerM = ossimImageHandlerRegistry::instance()->open(theLastMaster);
//			if (!handlerM)
//			{
//				cerr<<"ossimImageCorrelator"<<"::execute can't create handler for master image  "<< theLastMaster <<endl;
//				continue;
//			}
//			theMChain->add(handlerM.get());
//			// select only one band (if multiple)
//			ossim_uint32 mbc = handlerM->getNumberOfOutputBands();
//
//			//add a band selector
//			ossim_uint32 mb = getMasterBand();
//			if (mb>=mbc) 
//			{
//				cerr<<"ossimImageCorrelator"<<"::execute Warning not enough bands in master, only "<< mbc <<endl;
//				mb=0;
//			}
//			//cout<<"Using band "<<mb<<" for master"<<endl; //TBR
//			theMasterBandSelector = new ossimBandSelector;
//			theMasterBandSelector->connectMyInputTo(0, handlerM.get());
//			theMasterBandSelector->setOutputBandList(vector<ossim_uint32>(1,mb));
//			theMChain->add(theMasterBandSelector.get());
//
//			caster[1]->connectMyInputTo(0, theMasterBandSelector.get());
//			theMasterProjection = handlerM->getImageGeometry()->getProjection();
//
//			ossimIpt mul = slave2master(sul - delta_lr);
//			ossimIpt mlr = slave2master(slr + delta_lr);
//			ossimIrect mrect = ossimIrect(mul, mlr);
//
//			ossimImageCorrelator::runMatch(srect, mrect, tp);
//			if (match_result == match_state::slave_faild)
//			{
//				break;
//			}
//
//			if (thedTieptFilename != ossimFilename::NIL && tp.size() > 0)
//			{
//				//write on stream
//				writeTiePoints(tp);
//			}
//
//			if (getStoreFlag())
//			{
//				//store them : insert at the end (constant time) //TBD : conditional store
//				for (int iPoint = 0;iPoint < (int)tp.size();++iPoint)
//				{
//					theTiePoints.push_back(tp[iPoint]);
//
//					// convert "Image to Image" to "Ground to Image" tie points    //TBC : use more generic tie points
//					ossimRefPtr<ossimTieGpt> tgi(new ossimTieGpt);
//					//set master ground pos
//					theMasterProjection->lineSampleToWorld( tp[iPoint].getMasterPoint() , *tgi ); //TBC : is it always lon/lat WGS84?
//					double hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(*tgi);
//					tgi->setGroundPoint(ossimGpt(tgi->lon, tgi->lat, hgt));
//					//ossimDpt dpt = outProj->forward(*tgi);
//					//tgi->setGroundPoint(ossimGpt(dpt.x, dpt.y, hgt));
//					//set slave image position
//					tgi->refImagePoint() = tp[iPoint].getSlavePoint();
//					//set score
//					tgi->setScore(tp[iPoint].score);
//
//					//add to list
//					theTset.addTiePoint(tgi);
//				}
//				//theTiePoints.insert(theTiePoints.end(),tp.begin(),tp.end());
//			}
//
//			if (tp.size() > 0)
//			{
//				return true;
//			}
//		}
//		if (handlerM != NULL)
//		{
//			handlerM->disconnect();
//			handlerM = NULL;
//		}
//	}
//	
//	//setPercentComplete(100.0);
//	return false;
//}

//bool ossimImageCorrelator::getGridFeaturesParallel(const ossimIrect& rect)
//{
//	if (!theSlaveBandSelector)
//	{
//		ossimNotify(ossimNotifyLevel_WARN)
//			<< "WARN ossimTieGenerator::scanForEdges():"
//			<< "\nInput source is not a ossimImageChip.  Returning..." << std::endl;
//		return false;
//	}
//
//	const ossim_int32 TILE_HEIGHT	= theTileSize;
//	const ossim_int32 TILE_WIDTH	= theTileSize;
//	const ossim_int32 START_LINE = rect.ul().y;
//	const ossim_int32 STOP_LINE  = rect.lr().y;
//	const ossim_int32 START_SAMP = rect.ul().x;
//	const ossim_int32 STOP_SAMP  = rect.lr().x;
//	int nWidth = STOP_SAMP-START_SAMP;
//	int nHeight = STOP_LINE-START_LINE;
//
//	// For percent complete status.
//	ossim_int32 tilerows=(STOP_LINE-START_LINE+TILE_HEIGHT) / TILE_HEIGHT; //ceil : (stop-start+1+size-1)/size
//	ossim_int32 tilecols=(STOP_SAMP-START_SAMP+TILE_WIDTH) / TILE_WIDTH;
//	double total_tiles = ((double)tilerows)*tilecols;
//	double tiles_processed = 0.0;
//
//	// loop through all tiles
//	// need to use a sequencer for parallelism in the future TBD
//	ossim_int32 line=START_LINE;
//	ossim_int32 i,j;
//
//	vector<row_col> row_col_List;
//	double center_row = tilerows * 0.5;
//	double center_col = tilecols * 0.5;
//	for (i=0;(i<tilerows);++i)
//	{
//		for (j=0;(j<tilecols);++j )
//		{
//			row_col_List.push_back(row_col((center_row-i-1),(center_col-j-1)));
//		}
//	}
//
//	ossimRefPtr<ossimImageHandler> master_handler = NULL;
//	ossimProjection* master_projection = NULL;
//	ossimRefPtr<ossimBandSelector> master_bandselector = NULL;
//	cv::Mat slaveMat, masterMat;
//	ossimRefPtr<ossimCastTileSourceFilter> master_caster = new ossimCastTileSourceFilter();
//	master_caster->setOutputScalarType(OSSIM_FLOAT64);
//
//	std::sort(row_col_List.begin(), row_col_List.end(), RowColCompare);
//	for (int i=0;i < (int)row_col_List.size();++i)
//	{
//		int icol = floor(row_col_List[i].col_idx+center_col+0.5);
//		int irow = floor(row_col_List[i].row_idx+center_row+0.5);
//		ossim_int32 samp=START_SAMP+icol*TILE_WIDTH;
//		ossim_int32 line=START_LINE+irow*TILE_HEIGHT;
//
//
//		ossim_int32 BufHeight = TILE_HEIGHT;
//		ossim_int32 BufWidth = TILE_WIDTH;
//		//行末尾小块处理
//		if (irow == tilerows-1)
//		{
//			BufHeight = nHeight - (tilerows-1) * TILE_HEIGHT;//得出当前块的宽度Bufsizex，高度Bufsizey
//			BufHeight = min(BufHeight, TILE_HEIGHT);
//		}
//		//列末尾小块处理
//		if (icol == tilecols-1)
//		{
//			BufWidth = nWidth - (tilecols-1) * TILE_WIDTH;//得出当前块的宽度Bufsizex，高度Bufsizey
//			BufWidth = min(BufWidth, TILE_WIDTH);
//		}
//
//		vector<ossimTDpt> tp(1);
//		int match_result;
//
//		// slave
//		ossimIrect srect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1));
//		ossimIpt sul = srect.ul();
//		ossimIpt slr = srect.lr();
//		// master
//		ossimIpt delta_lr((ossim_int32)(ceil(theSlaveAccuracy)), (ossim_int32)(ceil(theSlaveAccuracy)) );
//		if (master_handler != NULL)
//		{
//			ossimIpt mul = slave2master(theSlaveProjection, master_projection, sul - delta_lr);
//			ossimIpt mlr = slave2master(theSlaveProjection, master_projection, slr + delta_lr);
//			ossimIrect mrect = ossimIrect(mul, mlr);
//			if(master_handler->getBoundingRect(0).pointWithin(mul) 
//				&& master_handler->getBoundingRect(0).pointWithin(mlr))
//			{
//				if (!createTileMat(caster[0], srect, slaveMat, 0))
//				{
//					continue;
//				}
//				if (!createTileMat(master_caster, mrect, masterMat, 0))
//				{
//					goto getmasters;
//				}	
//				match_result = runMatchParallel(slaveMat, masterMat, tp[0]);
//				if (match_result == match_state::slave_faild)
//				{
//					continue;
//				}
//				if (thedTieptFilename != ossimFilename::NIL && match_state::success == match_result)
//				{
//					tp[0].setMasterPoint(tp[0].getMasterPoint() + mul);
//					tp[0].setSlavePoint(tp[0].getSlavePoint() + sul);
//					//write on stream
//					writeTiePoints(tp);
//					if (getStoreFlag())
//					{
//						theTiePoints.push_back(tp[0]);
//						// convert "Image to Image" to "Ground to Image" tie points    //TBC : use more generic tie points
//
//						ossimRefPtr<ossimTieGpt> tgi(new ossimTieGpt);
//						//set master ground pos
//						master_projection->lineSampleToWorld( tp[0].getMasterPoint() , *tgi ); //TBC : is it always lon/lat WGS84?
//						double hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(*tgi);
//						//ossimDpt dpt = outProj->forward(*tgi);
//						tgi->setGroundPoint(ossimGpt(tgi->lon, tgi->lat, hgt));
//						//set slave image position
//						tgi->refImagePoint() = tp[0].getSlavePoint();
//						//set score
//						tgi->setScore(tp[0].score);
//
//						//add to list
//						theTset.addTiePoint(tgi);
//					}
//				}
//				if (match_state::success == match_result)
//				{
//					return true;
//				}
//			}
//		}
//getmasters:
//		// search in the reference library
//		ossimGpt ul_latlon, lr_latlon;
//		theSlaveProjection->lineSampleToWorld(sul-delta_lr, ul_latlon);
//		theSlaveProjection->lineSampleToWorld(slr+delta_lr, lr_latlon);
//		vector<ossimFilename> masterFileList;
//		if (theMaster.ext().upcase() == "SHP")
//		{
//			getMasterList(theMaster, masterFileList, ul_latlon, lr_latlon);
//		}
//		else{
//			masterFileList.clear();
//			masterFileList.push_back(theMaster);
//		}
//		for (int iFile = 0;iFile < (int)masterFileList.size();++iFile)
//		{
//			theLastMaster = masterFileList[iFile];
//
//			master_handler = ossimImageHandlerRegistry::instance()->open(theLastMaster);
//			if (!master_handler)
//			{
//				cerr<<"ossimImageCorrelator"<<"::execute can't create handler for master image  "<< theLastMaster <<endl;
//				continue;
//			}
//			// select only one band (if multiple)
//			ossim_uint32 mbc = master_handler->getNumberOfOutputBands();
//
//			//add a band selector
//			ossim_uint32 mb = getMasterBand();
//			if (mb>=mbc) 
//			{
//				cerr<<"ossimImageCorrelator"<<"::execute Warning not enough bands in master, only "<< mbc <<endl;
//				mb=0;
//			}
//			//cout<<"Using band "<<mb<<" for master"<<endl; //TBR
//			master_bandselector = new ossimBandSelector;
//			master_bandselector->connectMyInputTo(0, master_handler.get());
//			master_bandselector->setOutputBandList(vector<ossim_uint32>(1,mb));
//			master_caster->connectMyInputTo(0, master_bandselector.get());
//			master_projection = master_handler->getImageGeometry()->getProjection();
//
//			ossimIpt mul = slave2master(theSlaveProjection, master_projection, sul - delta_lr);
//			ossimIpt mlr = slave2master(theSlaveProjection, master_projection, slr + delta_lr);
//			ossimIrect mrect = ossimIrect(mul, mlr);
//
//			if (!createTileMat(caster[0], srect, slaveMat, 0))
//			{
//				break;;
//			}
//			if (!createTileMat(master_caster, mrect, masterMat, 0))
//			{
//				continue;;
//			}	
//			match_result = runMatchParallel(slaveMat, masterMat, tp[0]);
//			if (match_result == match_state::slave_faild)
//			{
//				break;;
//			}
//			if (thedTieptFilename != ossimFilename::NIL && match_state::success == match_result)
//			{
//				tp[0].setMasterPoint(tp[0].getMasterPoint() + mul);
//				tp[0].setSlavePoint(tp[0].getSlavePoint() + sul);
//				//write on stream
//				writeTiePoints(tp);
//				if (getStoreFlag())
//				{
//					theTiePoints.push_back(tp[0]);
//					// convert "Image to Image" to "Ground to Image" tie points    //TBC : use more generic tie points
//
//					ossimRefPtr<ossimTieGpt> tgi(new ossimTieGpt);
//					//set master ground pos
//					master_projection->lineSampleToWorld( tp[0].getMasterPoint() , *tgi ); //TBC : is it always lon/lat WGS84?
//					double hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(*tgi);
//					//ossimDpt dpt = outProj->forward(*tgi);
//					tgi->setGroundPoint(ossimGpt(tgi->lon, tgi->lat, hgt));
//					//set slave image position
//					tgi->refImagePoint() = tp[0].getSlavePoint();
//					//set score
//					tgi->setScore(tp[0].score);
//
//					//add to list
//					theTset.addTiePoint(tgi);
//				}
//			}
//			if (match_state::success == match_result)
//			{
//				return true;
//			}
//		}
//		//if (master_handler != NULL)
//		//{
//		//	master_handler->disconnect();
//		//	master_handler = NULL;
//		//}
//	}
//
//	//setPercentComplete(100.0);
//	return false;
////}

bool ossimImageCorrelator::getGridFeaturesParallel(const ossimIrect& rect)
{
	if (!theSlaveBandSelector)
	{
		ossimNotify(ossimNotifyLevel_WARN)
			<< "WARN ossimTieGenerator::scanForEdges():"
			<< "\nInput source is not a ossimImageChip.  Returning..." << std::endl;
		return false;
	}

	const ossim_int32 TILE_HEIGHT	= theTileSize;
	const ossim_int32 TILE_WIDTH	= theTileSize;
	const ossim_int32 START_LINE = rect.ul().y;
	const ossim_int32 STOP_LINE  = rect.lr().y;
	const ossim_int32 START_SAMP = rect.ul().x;
	const ossim_int32 STOP_SAMP  = rect.lr().x;
	int nWidth = STOP_SAMP-START_SAMP;
	int nHeight = STOP_LINE-START_LINE;

	// For percent complete status.
	ossim_int32 tilerows=(STOP_LINE-START_LINE+TILE_HEIGHT) / TILE_HEIGHT; //ceil : (stop-start+1+size-1)/size
	ossim_int32 tilecols=(STOP_SAMP-START_SAMP+TILE_WIDTH) / TILE_WIDTH;
	double total_tiles = ((double)tilerows)*tilecols;
	double tiles_processed = 0.0;

	// loop through all tiles
	// need to use a sequencer for parallelism in the future TBD
	ossim_int32 line=START_LINE;
	ossim_int32 i,j;

	vector<row_col> row_col_List;
	double center_row = tilerows * 0.5;
	double center_col = tilecols * 0.5;
	for (i=0;(i<tilerows);++i)
	{
		for (j=0;(j<tilecols);++j )
		{
			row_col_List.push_back(row_col((center_row-i-1),(center_col-j-1)));
		}
	}

	int ncore = omp_get_num_procs();//获取执行核的总数；  目前机器CPU的数量
	row_col_step = 5;
	std::sort(row_col_List.begin(), row_col_List.end(), RowColCompare);	

	//GdalRasterApp slaveApp;
	//slaveApp.open(theSlave.c_str());
	bool bDebug = true;
	//if (theTset.size() == 20)
	//{
	//	bDebug = true;
	//}
	bool found = false;
	int N_PARALLEL = 1;//ncore*2;
	for (int i=0;i < (int)row_col_List.size() && !found;)
	{
		int nParallel = min(N_PARALLEL, (int)row_col_List.size()-1-i);	// 保证末尾不越界

		std::vector<cv::Mat> slaveMatList;
		std::vector<ossimIrect> srectList;
		std::vector<bool>  slaveValidList(nParallel, false);
		for (int j = 0; j < nParallel;j++)
		{
			int icol = floor(row_col_List[i+j].col_idx+center_col+0.5);
			int irow = floor(row_col_List[i+j].row_idx+center_row+0.5);
			ossim_int32 samp=START_SAMP+icol*TILE_WIDTH;
			ossim_int32 line=START_LINE+irow*TILE_HEIGHT;


			ossim_int32 BufHeight = TILE_HEIGHT;
			ossim_int32 BufWidth = TILE_WIDTH;
			//行末尾小块处理
			if (irow == tilerows-1)
			{
				BufHeight = nHeight - (tilerows-1) * TILE_HEIGHT;//得出当前块的宽度Bufsizex，高度Bufsizey
				BufHeight = min(BufHeight, TILE_HEIGHT);
			}
			//列末尾小块处理
			if (icol == tilecols-1)
			{
				BufWidth = nWidth - (tilecols-1) * TILE_WIDTH;//得出当前块的宽度Bufsizex，高度Bufsizey
				BufWidth = min(BufWidth, TILE_WIDTH);
			}

			// slave
			cv::Mat slaveMat;
			ossimIrect srect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1));
			//if (slaveApp.getRect2CvMat( srect, slaveMat, theSlaveBand))
			//{
			//	slaveValidList[j] = true;
			//}
			{
			if (createTileMat(caster[0], srect, slaveMat, 0))
			{
				slaveValidList[j] = true;
			}
			}
			slaveMatList.push_back(slaveMat);
			srectList.push_back(srect);
		}
		for (int j = 0; j < nParallel;j++)
		{
			if (found || !slaveValidList[j])
			{
				continue;
			}
			ossimIpt sul = srectList[j].ul();
			ossimIpt slr = srectList[j].lr();
			ossimIpt delta_lr((ossim_int32)(ceil(theSlaveAccuracy)), (ossim_int32)(ceil(theSlaveAccuracy)) );

			vector<ossimTDpt> tp(1);
			// search in the reference library
			ossimGpt ul_latlon, lr_latlon;
			theSlaveProjection->lineSampleToWorld(sul-delta_lr, ul_latlon);
			theSlaveProjection->lineSampleToWorld(slr+delta_lr, lr_latlon);
			//ossimDpt tempDpt;
			//slaveApp.linesample2lonlat(sul-delta_lr, tempDpt);
			//ul_latlon = ossimGpt(tempDpt.y, tempDpt.x);
			//slaveApp.linesample2lonlat(slr+delta_lr, tempDpt);
			//lr_latlon = ossimGpt(tempDpt.y, tempDpt.x);
			vector<ossimFilename> masterFileList;
			if (theMaster.ext().upcase() == "SHP")
			{
				getMasterList(theMaster, masterFileList, ul_latlon, lr_latlon);
			}
			else{
				masterFileList.clear();
				masterFileList.push_back(theMaster);
			}

			//ossimRefPtr<ossimImageHandler> master_handler = NULL;
			//ossimProjection* master_projection = NULL;
			//ossimRefPtr<ossimBandSelector> master_bandselector = NULL;
			cv::Mat masterMat;
			//ossimRefPtr<ossimCastTileSourceFilter> master_caster = new ossimCastTileSourceFilter();
			//master_caster->setOutputScalarType(OSSIM_FLOAT64);
			//master_caster->setOutputScalarType(OSSIM_UCHAR);

			int match_result = match_state::success;
			for (int iFile = 0;iFile < (int)masterFileList.size();++iFile)
			{
				if (found || match_result == match_state::slave_faild)
				{
					continue;
				}

				ossimFilename lastMaster = masterFileList[iFile];
				theLastMaster = lastMaster;
				GdalRasterApp masterApp;
				if (!masterApp.open(lastMaster.c_str()))
				{
					cerr<<"ossimImageCorrelator"<<"::execute can't open master image  "<< lastMaster <<endl;
					continue;
				}
				// select only one band (if multiple)
				ossim_uint32 mbc = masterApp.nBand();

				//add a band selector
				ossim_uint32 mb = theMasterBand;
				if (mb>=mbc) 
				{
					cerr<<"ossimImageCorrelator"<<"::execute Warning not enough bands in master, only "<< mbc <<endl;
					mb=0;
				}

				// master
				ossimIpt mul, mlr;
				ossimGpt tempGpt;
				theSlaveProjection->lineSampleToWorld(sul - delta_lr, tempGpt);
				masterApp.lonlat2linesample(ossimDpt(tempGpt.lon, tempGpt.lat), mul);
				theSlaveProjection->lineSampleToWorld(slr + delta_lr, tempGpt);
				masterApp.lonlat2linesample(ossimDpt(tempGpt.lon, tempGpt.lat), mlr);
				//ossimDpt tempDpt;
				//slaveApp.linesample2lonlat(sul - delta_lr, tempDpt);
				//masterApp.lonlat2linesample(tempDpt, mul);
				//slaveApp.linesample2lonlat(slr + delta_lr, tempDpt);
				//masterApp.lonlat2linesample(tempDpt, mlr);
				ossimIrect mrect = ossimIrect(mul, mlr);
				if(!masterApp.getBoundary().pointWithin(mul) 
					|| !masterApp.getBoundary().pointWithin(mlr))
				{
					masterApp.close();
					continue;
				}
				if (!masterApp.getRect2CvMat( mrect, masterMat, mb))
				{
					masterApp.close();
					continue;
				}
				match_result = runMatchParallel(slaveMatList[j], masterMat, tp[0], this, bDebug);
				if (match_result == match_state::slave_faild)
				{
					masterApp.close();
					continue;
				}
				if (theFilename != ossimFilename::NIL && match_state::success == match_result)
				{
					//ossimTieGpt tiePt;
					//tiePt.setImagePoint(ossimDpt(tp[0].getSlavePoint() + sul));
					//ossimGpt gpt;
					//masterApp.linesample2lonlat(tp[0].getMasterPoint() + mul, gpt);
					//tiePt.setGroundPoint(gpt);
					//theTset.addTiePoint(&tiePt);
					tp[0].setMasterPoint(tp[0].getMasterPoint() + mul);
					tp[0].setSlavePoint(tp[0].getSlavePoint() + sul);

					// convert "Image to Image" to "Ground to Image" tie points    //TBC : use more generic tie points
					ossimRefPtr<ossimTieGpt> tgi(new ossimTieGpt);
					//set master ground pos
					ossimDpt lonlat;
					masterApp.linesample2lonlat(tp[0].getMasterPoint(), lonlat);
					double hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(ossimGpt(lonlat.y, lonlat.x));
					//ossimDpt dpt = outProj->forward(*tgi);
					tgi->setGroundPoint(ossimGpt(lonlat.y, lonlat.x, hgt));
					//set slave image position
					tgi->refImagePoint() = tp[0].getSlavePoint();
					//set score
					tgi->setScore(tp[0].score);

					//add to list
					theTset.addTiePoint(tgi);
				}
				if (match_state::success == match_result)
				{
					masterApp.close();
					found = true;
					continue;
				}
				masterApp.close();
			}
		}
		slaveMatList.clear();
		slaveValidList.clear();
		srectList.clear();
		i += N_PARALLEL;
	}
	//slaveApp.close();
	return found;
}

//bool ossimImageCorrelator::getGridFeaturesParallel(const ossimIrect& rect)
//{
//	if (!theSlaveBandSelector)
//	{
//		ossimNotify(ossimNotifyLevel_WARN)
//			<< "WARN ossimTieGenerator::scanForEdges():"
//			<< "\nInput source is not a ossimImageChip.  Returning..." << std::endl;
//		return false;
//	}
//
//	const ossim_int32 TILE_HEIGHT	= theTileSize;
//	const ossim_int32 TILE_WIDTH	= theTileSize;
//	const ossim_int32 START_LINE = rect.ul().y;
//	const ossim_int32 STOP_LINE  = rect.lr().y;
//	const ossim_int32 START_SAMP = rect.ul().x;
//	const ossim_int32 STOP_SAMP  = rect.lr().x;
//	int nWidth = STOP_SAMP-START_SAMP;
//	int nHeight = STOP_LINE-START_LINE;
//
//	// For percent complete status.
//	ossim_int32 tilerows=(STOP_LINE-START_LINE+TILE_HEIGHT) / TILE_HEIGHT; //ceil : (stop-start+1+size-1)/size
//	ossim_int32 tilecols=(STOP_SAMP-START_SAMP+TILE_WIDTH) / TILE_WIDTH;
//	double total_tiles = ((double)tilerows)*tilecols;
//	double tiles_processed = 0.0;
//
//	// loop through all tiles
//	// need to use a sequencer for parallelism in the future TBD
//	ossim_int32 line=START_LINE;
//	ossim_int32 i,j;
//
//	vector<row_col> row_col_List;
//	double center_row = tilerows * 0.5;
//	double center_col = tilecols * 0.5;
//	for (i=0;(i<tilerows);++i)
//	{
//		for (j=0;(j<tilecols);++j )
//		{
//			row_col_List.push_back(row_col((center_row-i-1),(center_col-j-1)));
//		}
//	}
//
//	int ncore = omp_get_num_procs();//获取执行核的总数；  目前机器CPU的数量
//	row_col_step = 5;
//	std::sort(row_col_List.begin(), row_col_List.end(), RowColCompare);	
//
//	//GdalRasterApp slaveApp;
//	//slaveApp.open(theSlave.c_str());
//	bool bDebug = true;
//	//if (theTset.size() == 20)
//	//{
//	//	bDebug = true;
//	//}
//	bool found = false;
//	int N_PARALLEL = 2;//ncore*2;
//	for (int i=0;i < (int)row_col_List.size() && !found;)
//	{
//		int nParallel = min(N_PARALLEL, (int)row_col_List.size()-1-i);	// 保证末尾不越界
//
//		std::vector<cv::Mat> slaveMatList;
//		std::vector<ossimIrect> srectList;
//		std::vector<bool>  slaveValidList(nParallel, false);
//		for (int j = 0; j < nParallel;j++)
//		{
//			int icol = floor(row_col_List[i+j].col_idx+center_col+0.5);
//			int irow = floor(row_col_List[i+j].row_idx+center_row+0.5);
//			ossim_int32 samp=START_SAMP+icol*TILE_WIDTH;
//			ossim_int32 line=START_LINE+irow*TILE_HEIGHT;
//
//
//			ossim_int32 BufHeight = TILE_HEIGHT;
//			ossim_int32 BufWidth = TILE_WIDTH;
//			//行末尾小块处理
//			if (irow == tilerows-1)
//			{
//				BufHeight = nHeight - (tilerows-1) * TILE_HEIGHT;//得出当前块的宽度Bufsizex，高度Bufsizey
//				BufHeight = min(BufHeight, TILE_HEIGHT);
//			}
//			//列末尾小块处理
//			if (icol == tilecols-1)
//			{
//				BufWidth = nWidth - (tilecols-1) * TILE_WIDTH;//得出当前块的宽度Bufsizex，高度Bufsizey
//				BufWidth = min(BufWidth, TILE_WIDTH);
//			}
//
//			// slave
//			cv::Mat slaveMat;
//			ossimIrect srect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1));
//			//if (slaveApp.getRect2CvMat( srect, slaveMat, theSlaveBand))
//			//{
//			//	slaveValidList[j] = true;
//			//}
//			if (createTileMat(caster[0], srect, slaveMat, 0))
//			{
//				slaveValidList[j] = true;
//			}
//			slaveMatList.push_back(slaveMat);
//			srectList.push_back(srect);
//		}
//
//		ossimFilename lastMaster = theMaster;
//		theLastMaster = lastMaster;
//		GdalRasterApp masterApp;
//		if (!masterApp.open(lastMaster.c_str()))
//		{
//			cerr<<"ossimImageCorrelator"<<"::execute can't open master image  "<< lastMaster <<endl;
//			continue;
//		}
//		// select only one band (if multiple)
//		ossim_uint32 mbc = masterApp.nBand();
//
//		//add a band selector
//		ossim_uint32 mb = theMasterBand;
//		if (mb>=mbc) 
//		{
//			cerr<<"ossimImageCorrelator"<<"::execute Warning not enough bands in master, only "<< mbc <<endl;
//			mb=0;
//		}
//
//		int match_result = match_state::match_failed;
//#pragma omp parallel for num_threads(nParallel)
//		for (int j = 0; j < nParallel;j++)
//		{
//			if (found || !slaveValidList[j])
//			{
//				continue;
//			}
//			ossimIpt sul = srectList[j].ul();
//			ossimIpt slr = srectList[j].lr();
//			ossimIpt delta_lr((ossim_int32)(ceil(theSlaveAccuracy)), (ossim_int32)(ceil(theSlaveAccuracy)) );
//
//			vector<ossimTDpt> tp(1);
//			// search in the reference library
//			ossimGpt ul_latlon, lr_latlon;
//			theSlaveProjection->lineSampleToWorld(sul-delta_lr, ul_latlon);
//			theSlaveProjection->lineSampleToWorld(slr+delta_lr, lr_latlon);
//			
//
//			cv::Mat masterMat;
//			
//
//			// master
//			ossimIpt mul, mlr;
//			ossimGpt tempGpt;
//			theSlaveProjection->lineSampleToWorld(sul - delta_lr, tempGpt);
//			masterApp.lonlat2linesample(ossimDpt(tempGpt.lon, tempGpt.lat), mul);
//			theSlaveProjection->lineSampleToWorld(slr + delta_lr, tempGpt);
//			masterApp.lonlat2linesample(ossimDpt(tempGpt.lon, tempGpt.lat), mlr);
//			//ossimDpt tempDpt;
//			//slaveApp.linesample2lonlat(sul - delta_lr, tempDpt);
//			//masterApp.lonlat2linesample(tempDpt, mul);
//			//slaveApp.linesample2lonlat(slr + delta_lr, tempDpt);
//			//masterApp.lonlat2linesample(tempDpt, mlr);
//			ossimIrect mrect = ossimIrect(mul, mlr);
//			if(!masterApp.getBoundary().pointWithin(mul) 
//				|| !masterApp.getBoundary().pointWithin(mlr))
//			{
//				continue;
//			}
//			if (!masterApp.getRect2CvMat( mrect, masterMat, mb))
//			{
//				continue;
//			}
//			match_result = runMatchParallel(slaveMatList[j], masterMat, tp[0], this, bDebug);
//			if (match_result == match_state::slave_faild)
//			{
//				continue;
//			}
//			if (thedTieptFilename != ossimFilename::NIL && match_state::success == match_result)
//			{
//				tp[0].setMasterPoint(tp[0].getMasterPoint() + mul);
//				tp[0].setSlavePoint(tp[0].getSlavePoint() + sul);
//				//write on stream
//				writeTiePoints(tp);
//				if (getStoreFlag())
//				{
//					theTiePoints.push_back(tp[0]);
//					// convert "Image to Image" to "Ground to Image" tie points    //TBC : use more generic tie points
//
//					ossimRefPtr<ossimTieGpt> tgi(new ossimTieGpt);
//					//set master ground pos
//					ossimDpt lonlat;
//					masterApp.linesample2lonlat(tp[0].getMasterPoint(), lonlat);
//					double hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(ossimGpt(lonlat.y, lonlat.x));
//					//ossimDpt dpt = outProj->forward(*tgi);
//					tgi->setGroundPoint(ossimGpt(lonlat.y, lonlat.x, hgt));
//					//set slave image position
//					tgi->refImagePoint() = tp[0].getSlavePoint();
//					//set score
//					tgi->setScore(tp[0].score);
//
//					//add to list
//					theTset.addTiePoint(tgi);
//				}
//			}
//			if (match_state::success == match_result)
//			{
//				found = true;
//				continue;
//			}
//			
//		}
//		slaveMatList.clear();
//		slaveValidList.clear();
//		srectList.clear();
//		i += N_PARALLEL;
//	}
//	//slaveApp.close();
//	return found;
//}

//bool ossimImageCorrelator::getGridFeaturesParallel(const ossimIrect& rect, void *pData)
//{
//	ossimImageCorrelator* pThis = (ossimImageCorrelator*)pData;
//	if (!pThis->theSlaveBandSelector)
//	{
//		ossimNotify(ossimNotifyLevel_WARN)
//			<< "WARN ossimTieGenerator::scanForEdges():"
//			<< "\nInput source is not a ossimImageChip.  Returning..." << std::endl;
//		return false;
//	}
//
//	const ossim_int32 TILE_HEIGHT	= pThis->theTileSize;
//	const ossim_int32 TILE_WIDTH	= pThis->theTileSize;
//	const ossim_int32 START_LINE = rect.ul().y;
//	const ossim_int32 STOP_LINE  = rect.lr().y;
//	const ossim_int32 START_SAMP = rect.ul().x;
//	const ossim_int32 STOP_SAMP  = rect.lr().x;
//	int nWidth = STOP_SAMP-START_SAMP;
//	int nHeight = STOP_LINE-START_LINE;
//
//	// For percent complete status.
//	ossim_int32 tilerows=(STOP_LINE-START_LINE+TILE_HEIGHT) / TILE_HEIGHT; //ceil : (stop-start+1+size-1)/size
//	ossim_int32 tilecols=(STOP_SAMP-START_SAMP+TILE_WIDTH) / TILE_WIDTH;
//	double total_tiles = ((double)tilerows)*tilecols;
//	double tiles_processed = 0.0;
//
//	// loop through all tiles
//	// need to use a sequencer for parallelism in the future TBD
//	ossim_int32 line=START_LINE;
//	ossim_int32 i,j;
//
//	vector<row_col> row_col_List;
//	double center_row = tilerows * 0.5;
//	double center_col = tilecols * 0.5;
//	for (i=0;(i<tilerows);++i)
//	{
//		for (j=0;(j<tilecols);++j )
//		{
//			row_col_List.push_back(row_col((center_row-i-1),(center_col-j-1)));
//		}
//	}
//
//	row_col_step = 5;
//	std::sort(row_col_List.begin(), row_col_List.end(), RowColCompare);	
//
//	bool bDebug = false;
//	//if (theTset.size() == 20)
//	//{
//	//	bDebug = true;
//	//}
//	bool found = false;
//	int ncore = omp_get_num_procs();//获取执行核的总数；  目前机器CPU的数量
//	//#pragma omp parallel for num_threads(ncore*2) shared(found)
////#pragma omp single
////{
//	for (int i=0;i < (int)row_col_List.size();i+=1)
//	{
//		if (found)
//		{
//			continue;
//		}
//		int icol = floor(row_col_List[i].col_idx+center_col+0.5);
//		int irow = floor(row_col_List[i].row_idx+center_row+0.5);
//		ossim_int32 samp=START_SAMP+icol*TILE_WIDTH;
//		ossim_int32 line=START_LINE+irow*TILE_HEIGHT;
//
//
//		ossim_int32 BufHeight = TILE_HEIGHT;
//		ossim_int32 BufWidth = TILE_WIDTH;
//		//行末尾小块处理
//		if (irow == tilerows-1)
//		{
//			BufHeight = nHeight - (tilerows-1) * TILE_HEIGHT;//得出当前块的宽度Bufsizex，高度Bufsizey
//			BufHeight = min(BufHeight, TILE_HEIGHT);
//		}
//		//列末尾小块处理
//		if (icol == tilecols-1)
//		{
//			BufWidth = nWidth - (tilecols-1) * TILE_WIDTH;//得出当前块的宽度Bufsizex，高度Bufsizey
//			BufWidth = min(BufWidth, TILE_WIDTH);
//		}
//		
//		// slave
//		cv::Mat slaveMat;
//		ossimIrect srect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1));
//		ossimIpt sul = srect.ul();
//		ossimIpt slr = srect.lr();
//		ossimIpt delta_lr((ossim_int32)(ceil(pThis->theSlaveAccuracy)), (ossim_int32)(ceil(pThis->theSlaveAccuracy)) );
//		if (!pThis->createTileMat(pThis->caster[0], srect, slaveMat, 0))
//		{
//			continue;
//		}
//
//		vector<ossimTDpt> tp(1);
//		// search in the reference library
//		ossimGpt ul_latlon, lr_latlon;
//		pThis->theSlaveProjection->lineSampleToWorld(sul-delta_lr, ul_latlon);
//		pThis->theSlaveProjection->lineSampleToWorld(slr+delta_lr, lr_latlon);
//		vector<ossimFilename> masterFileList;
//		if (pThis->theMaster.ext().upcase() == "SHP")
//		{
//			pThis->getMasterList(pThis->theMaster, masterFileList, ul_latlon, lr_latlon);
//		}
//		else{
//			masterFileList.clear();
//			masterFileList.push_back(pThis->theMaster);
//		}
//
//		ossimRefPtr<ossimImageHandler> master_handler = NULL;
//		ossimProjection* master_projection = NULL;
//		ossimRefPtr<ossimBandSelector> master_bandselector = NULL;
//		cv::Mat masterMat;
//		ossimRefPtr<ossimCastTileSourceFilter> master_caster = new ossimCastTileSourceFilter();
//		//master_caster->setOutputScalarType(OSSIM_FLOAT64);
//		master_caster->setOutputScalarType(OSSIM_UCHAR);
//		int match_result = match_state::success;
//		int nThreads = min(ncore*2, (int)masterFileList.size());
//		//int match_result = match_state::success;
////#pragma omp critical
////#pragma omp parallel for num_threads(nThreads) firstprivate(slaveMat) \
//		shared(found, match_result)\
//		private(master_handler, master_projection, master_bandselector, masterMat, master_caster)
//#pragma omp parallel for num_threads(nThreads)
//		for (int iFile = 0;iFile < (int)masterFileList.size();++iFile)
//		{
//			if (found || match_result == match_state::slave_faild)
//			{
//				continue;
//			}
//			//if (!master_caster)
//			//{
//			//	master_caster = new ossimCastTileSourceFilter();
//			//	master_caster->setOutputScalarType(OSSIM_FLOAT64);
//			//}
//			ossimFilename theLastMaster = masterFileList[iFile];
//			master_handler = ossimImageHandlerRegistry::instance()->open(theLastMaster);
//			pThis->theLastMaster = theLastMaster;
//			if (!master_handler)
//			{
//				cerr<<"ossimImageCorrelator"<<"::execute can't create handler for master image  "<< theLastMaster <<endl;
//				continue;
//			}
//			// select only one band (if multiple)
//			ossim_uint32 mbc = master_handler->getNumberOfOutputBands();
//
//			//add a band selector
//			ossim_uint32 mb = pThis->theMasterBand;
//			if (mb>=mbc) 
//			{
//				cerr<<"ossimImageCorrelator"<<"::execute Warning not enough bands in master, only "<< mbc <<endl;
//				mb=0;
//			}
//			//cout<<"Using band "<<mb<<" for master"<<endl; //TBR
//			master_bandselector = new ossimBandSelector;
//			master_bandselector->connectMyInputTo(0, master_handler.get());
//			master_bandselector->setOutputBandList(vector<ossim_uint32>(1,mb));
//			master_caster->connectMyInputTo(0, master_bandselector.get());
//			master_projection = master_handler->getImageGeometry()->getProjection();
//
//			// master
//			ossimIpt mul = pThis->slave2master(pThis->theSlaveProjection, master_projection, sul - delta_lr);
//			ossimIpt mlr = pThis->slave2master(pThis->theSlaveProjection, master_projection, slr + delta_lr);
//			ossimIrect mrect = ossimIrect(mul, mlr);
//			if(!master_handler->getBoundingRect(0).pointWithin(mul) 
//				|| !master_handler->getBoundingRect(0).pointWithin(mlr))
//			{
//				continue;
//			}
//			if (!createTileMat(master_caster, mrect, masterMat, 0))
//			{
//				//master_caster->disconnect();
//				//if (master_handler != NULL)
//				//{
//				//	master_handler->disconnect();
//				//	master_handler = NULL;
//				//}
//				continue;
//			}
//			match_result = pThis->runMatchParallel(slaveMat, masterMat, tp[0], pData, bDebug);
//			if (match_result == match_state::slave_faild)
//			{
//				//master_caster->disconnect();
//				//if (master_handler != NULL)
//				//{
//				//	master_handler->disconnect();
//				//	master_handler = NULL;
//				//}
//				continue;
//			}
//			if (pThis->thedTieptFilename != ossimFilename::NIL && match_state::success == match_result)
//			{
//				tp[0].setMasterPoint(tp[0].getMasterPoint() + mul);
//				tp[0].setSlavePoint(tp[0].getSlavePoint() + sul);
//				//write on stream
//				pThis->writeTiePoints(tp);
//				if (pThis->getStoreFlag())
//				{
//					pThis->theTiePoints.push_back(tp[0]);
//					// convert "Image to Image" to "Ground to Image" tie points    //TBC : use more generic tie points
//
//					ossimRefPtr<ossimTieGpt> tgi(new ossimTieGpt);
//					//set master ground pos
//					master_projection->lineSampleToWorld( tp[0].getMasterPoint() , *tgi ); //TBC : is it always lon/lat WGS84?
//					double hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(*tgi);
//					//ossimDpt dpt = outProj->forward(*tgi);
//					tgi->setGroundPoint(ossimGpt(tgi->lat, tgi->lon, hgt));
//					//set slave image position
//					tgi->refImagePoint() = tp[0].getSlavePoint();
//					//set score
//					tgi->setScore(tp[0].score);
//
//					//add to list
//					pThis->theTset.addTiePoint(tgi);
//				}
//			}
//			if (match_state::success == match_result)
//			{
//				master_caster->disconnect();
//				if (master_handler != NULL)
//				{
//					master_handler->disconnect();
//					master_handler = NULL;
//				}
//
//				found = true;
//				//return true;
//				continue;
//				//exit(0);
//				//found = true;
//				//break;
//				//return true;
//			}
//
//			//master_caster->disconnect();
//			//if (master_handler != NULL)
//			//{
//			//	master_handler->disconnect();
//			//	master_handler = NULL;
//			//}
//		}
//
//		master_caster->disconnect();
//		if (master_handler != NULL)
//		{
//			master_handler->disconnect();
//			master_handler = NULL;
//		}
//	}
////}
//	//setPercentComplete(100.0);
//	return found;
//}

bool ossimImageCorrelator::getAllFeatures()
{
	static const char MODULE[] = "ossimTieGenerator::getAllFeatures";

	if (traceDebug()) CLOG << " Entered..." << endl;

	if (!theSlaveBandSelector)
	{
		ossimNotify(ossimNotifyLevel_WARN)
			<< "WARN ossimTieGenerator::scanForEdges():"
			<< "\nInput source is not a ossimImageChip.  Returning..." << std::endl;
		return false;
	}

	int nWidth = theAreaOfInterest.width();
	int nHeight =theAreaOfInterest.height();
	int nPointRequired = thePointNumber;
	// Some constants needed throughout...
	const ossim_int32 START_LINE = theAreaOfInterest.ul().y;
	const ossim_int32 STOP_LINE  = theAreaOfInterest.lr().y;
	const ossim_int32 START_SAMP = theAreaOfInterest.ul().x;
	const ossim_int32 STOP_SAMP  = theAreaOfInterest.lr().x;

	// For percent complete status.
	ossim_int32 tilerows = ceil(sqrt(nPointRequired * nHeight / (double)nWidth ));
	ossim_int32 tilecols = ceil(sqrt(nPointRequired * nWidth / (double)nHeight ));
	const ossim_int32 TILE_HEIGHT    = ceil(nHeight / (double)tilerows);
	const ossim_int32 TILE_WIDTH     = ceil(nWidth / (double)tilecols);
	double total_tiles = ((double)tilerows)*tilecols;
	double total_tiles_processed;
	double tiles_processed = 0.0;
	// Set the status message to be "scanning source for edges..."
#if OSSIM_HAS_MPI
	if (ossimMpi::instance()->getRank() == 0)
#endif
	{
		ossimNotify(ossimNotifyLevel_INFO) << "Getting tie points..." << std::endl;
	}


	// loop through all tiles
	// need to use a sequencer for parallelism in the future TBD
	theTiePoints.clear();

	vector<row_col> row_col_List;
	for (int i=0;i<tilerows;++i)
	{
		for (int j=0;j<tilecols;++j )
		{
			row_col_List.push_back(row_col(i,j));
		}
	}

	// Start off with a percent complete at 0...
	setPercentComplete(0.0);
#if OSSIM_HAS_MPI
	MPI_Bcast(&tiles_processed, 1, MPI_DOUBLE, 0, MPI_COMM_WORLD);
	int myid = ossimMpi::instance()->getRank();
	int numprocs = ossimMpi::instance()->getNumberOfProcessors();
#else
	int myid = 0;
	int numprocs = 1;
#endif
	ossimRefPtr<ossimCastTileSourceFilter> slaveCaster = caster[0];
	for (int i = myid;i < (int)row_col_List.size(); i += numprocs)
	{
		int irow = (int)row_col_List[i].row_idx;

		ossim_int32 line=START_LINE+irow*TILE_HEIGHT;
		ossim_int32 BufHeight = TILE_HEIGHT;
		//列末尾小块处理
		if (irow == tilerows-1)
		{
			BufHeight = nHeight - (tilerows-1) * TILE_HEIGHT;
			BufHeight = min(BufHeight, TILE_HEIGHT);
		}

		int icol = (int)row_col_List[i].col_idx;
		ossim_int32 samp=START_SAMP+icol*TILE_WIDTH;
		ossim_int32 BufWidth = TILE_WIDTH;
		//列末尾小块处理
		if (icol == tilecols-1)
		{
			BufWidth = nWidth - (tilecols-1) * TILE_WIDTH;
			BufWidth = min(BufWidth, TILE_WIDTH);
		}
		// Get the tie points
		getGridFeaturesParallel(ossimIrect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1)));

		// Set the percent complete.
		tiles_processed += 1.0;
		//MPI_Reduce(&tiles_processed, 
		//	&total_tiles_processed, 
		//	1, 
		//	MPI_DOUBLE, 
		//	MPI_SUM, 
		//	0, 
		//	MPI_COMM_WORLD);
		if (myid == 0)
		{
			setPercentComplete((i+1)/total_tiles*100.0);
		}
		//if (i+numprocs > (int)row_col_List.size())
		//{
		//	MPI_Barrier(MPI_COMM_WORLD);
		//}
		
		fflush( stdout );
	}

#if OSSIM_HAS_MPI
	MPI_Barrier(MPI_COMM_WORLD);
#endif
	//writeTiePoints(theTset);
//	ossim_int32 i,j;
//#pragma omp parallel for
//	//for (i=0;(i<tilerows)&&!needsAborting();++i)
//	for (i=0;i<tilerows;++i)
//	{
//		ossim_int32 line=START_LINE+i*TILE_HEIGHT;
//		ossim_int32 BufHeight = TILE_HEIGHT;
//		//列末尾小块处理
//		if (i == tilerows-1)
//		{
//			BufHeight = nHeight - (tilerows-1) * TILE_HEIGHT;//得出当前块的宽度Bufsizex，高度Bufsizey
//			BufHeight = min(BufHeight, TILE_HEIGHT);
//		}
//		//for (j=0;(j<tilecols)&&!needsAborting();++j )
//		for (j=0;j<tilecols;++j )
//		{
//			ossim_int32 samp=START_SAMP+j*TILE_WIDTH;
//			ossim_int32 BufWidth = TILE_WIDTH;
//			//列末尾小块处理
//			if (j == tilecols-1)
//			{
//				BufWidth = nWidth - (tilecols-1) * TILE_WIDTH;//得出当前块的宽度Bufsizex，高度Bufsizey
//				BufWidth = min(BufWidth, TILE_WIDTH);
//			}
//			// Get the tie points
//
//#pragma omp critical
//			{
//				getGridFeaturesParallel(ossimIrect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1)));
//			}
//
//			// Set the percent complete.
//			tiles_processed += 1.0;
//			setPercentComplete(tiles_processed/total_tiles*100.0);
//
//		}
	//	}
	if (myid == 0)
	{
		setPercentComplete(100.0);
	}
	if (traceDebug()) CLOG << " Exited." << endl;
	return true;
}

void ossimImageCorrelator::writeTiePoints(const ossimTieGptSet& tp)
{
	ossimMapProjection* outProjection = ossimImageCorrelator::getOutputProjection();
	vector< ossimRefPtr<ossimTieGpt> >::const_iterator it;
	int icount = 0;
	for (it = tp.getTiePoints().begin();it!=tp.getTiePoints().end();++it)
	{
		char buf[1024];
		double hgt = ossimElevManager::instance()->getHeightAboveEllipsoid((*it)->getGroundPoint());
		ossimDpt dpt = (*it)->getImagePoint();
		ossimGpt gpt = (*it)->getGroundPoint();
		if (!outProjection->isGeographic())
		{
			ossimDpt d1 = outProjection->forward(gpt);
			gpt = ossimGpt(d1.x, d1.y, hgt);
		}
		sprintf_s(buf, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\n\0", 
			icount+=1,
			dpt.x, 
			dpt.y,
			gpt.lon,
			gpt.lat,
			hgt);
#if OSSIM_HAS_MPI
		int nLength = strlen(buf);
		MPIO_Request request;
		//MPI_File_iwrite_shared(theFileStream,
		//	buf,
		//	nLength,
		//	MPI_CHAR,
		//	&request);
		MPI_Status status;
		MPI_File_write_shared(theFileStream,
			buf,
			nLength,
			MPI_CHAR,
			&status);
#else
		theFileStream<<buf<<endl;
#endif
	}
}

void ossimImageCorrelator::setOutputName(const ossimString& filename)
{
	ossimOutputSource::setOutputName(filename);

	if (isOpen()) close();

	if (filename != "")
	{
		theFilename = filename;
	}
}

void ossimImageCorrelator::setAreaOfInterest(const ossimIrect& rect)
{
	theAreaOfInterest = rect;
}

bool ossimImageCorrelator::isOpen()const
{
#if OSSIM_HAS_MPI
	return (theFileStream != NULL);
#else
	return const_cast<fstream*>(&theFileStream)->is_open();
#endif
}

bool ossimImageCorrelator::open()
{
	if(isOpen())
	{
		close();
	}

	if (theFilename == ossimFilename::NIL)
	{
		return false;
	}
#if OSSIM_HAS_MPI
	MPI_File_delete((char*)theFilename.c_str(), MPI_INFO_NULL);
	MPI_File_open(MPI_COMM_WORLD,
		(char*)theFilename.c_str(),
		MPI_MODE_CREATE |  MPI_MODE_WRONLY,
		MPI_INFO_NULL,
		&theFileStream);
#else
	theFileStream.open(theFilename.c_str());
#endif
	return (theFileStream != NULL);
}

void ossimImageCorrelator::close()
{
	//if (isOpen()) theFileStream.close();
	if (isOpen())
	{
#if OSSIM_HAS_MPI
		MPI_File_close(&theFileStream);
		theFileStream = NULL;
		if (ossimMpi::instance()->getRank() == 0)
#else
		theFileStream.close();
#endif
		{
			fstream ofs;
			ofs.open(theFilename.setExtension("geom").c_str(), ios_base::out);
			ossimKeywordlist prjKwl;
			getOutputProjection()->saveState(prjKwl);
			ofs<<prjKwl;
			ofs.close();
		}
	}
}

void ossimImageCorrelator::setProperty(ossimRefPtr<ossimProperty> property)
{
   if(!property.valid())
   {
      return;
   }
   
   ossimString name = property->getName();
   
   if(name == "master_filename")
   {
      setMaster(ossimFilename(property->valueToString()));
   }
   else if(name == "slave_filename")
   {
      setSlave(ossimFilename(property->valueToString()));
   }
   else if(name == "projection_filename")
   {
	   setProjectionFile(ossimFilename(property->valueToString()));
   }
   else if(name == "master_band")
   {
      setMasterBand(property->valueToString().toUInt32());
   }
   else if(name == "slave_band")
   {
      setSlaveBand(property->valueToString().toUInt32());
   }
   else if(name == "scale_ratio")
   {
      setScaleRatio(property->valueToString().toFloat64());
   }
   else if(name == "point_number")
   {
	   setPointNumber(property->valueToString().toUInt32());
   }
   else if(name == "slave_accuracy")
   {
      setSlaveAccuracy(property->valueToString().toFloat64());
   }
   else if(name == "projection_type")
   {
      setProjectionType(property->valueToString());
   }
   else if(name == "output_filename")
   {
      setOutputName(property->valueToString());
   }
   else if(name == "tile_size")
   {
	   setTileSize(property->valueToString().toFloat64());
   }
   else if(name == "sift_nfeatures")
   {
	   setSiftNfeatures(property->valueToString().toInt());
   }
   else if(name == "sift_noctavelayers")
   {
	   setSiftNOctaveLayers(property->valueToString().toInt());
   }
   else if(name == "sift_contrastthreshold")
   {
	   setSiftContrastThreshold(property->valueToString().toFloat64());
   }
   else if(name == "sift_edgethreshold")
   {
	   setSiftEdgeThreshold(property->valueToString().toFloat64());
   }
   else if(name == "sift_sigma")
   {
	   setSiftSigma(property->valueToString().toFloat64());
   }
   else
   {
      ossimOutputSource::setProperty(property);
   }
}

ossimRefPtr<ossimProperty> ossimImageCorrelator::getProperty(const ossimString& name)const
{
   ossimRefPtr<ossimProperty> result = 0;
   
   if(name == "master_filename")
   {
      ossimFilenameProperty* filenameProp =
         new ossimFilenameProperty(name, getMaster());
      
      filenameProp->setIoType(ossimFilenameProperty::ossimFilenamePropertyIoType_INPUT);
      
      return filenameProp;
   }
   else if(name == "slave_filename")
   {
      ossimFilenameProperty* filenameProp =
         new ossimFilenameProperty(name, getSlave());
      
      filenameProp->setIoType(ossimFilenameProperty::ossimFilenamePropertyIoType_INPUT);
      
      return filenameProp;
   }
   else if(name == "projection_filename")
   {
	   return new ossimStringProperty(name, ossimString::toString(getProjectionFile()));
   }
   else if(name == "master_band")
   {
      return new ossimStringProperty(name, ossimString::toString(getMasterBand()));
   }
   else if(name == "slave_band")
   {
      return new ossimStringProperty(name, ossimString::toString(getSlaveBand()));
   }
   else if(name == "scale_ratio")
   {
      return new ossimStringProperty(name, ossimString::toString(getScaleRatio()));
   }
   else if(name == "point_number")
   {
	   return new ossimStringProperty(name, ossimString::toString(getPointNumber()));
   }
   else if(name == "slave_accuracy")
   {
      return new ossimStringProperty(name, ossimString::toString(getSlaveAccuracy()));
   }
   else if(name == "projection_type")
   {
      return new ossimStringProperty(name, getProjectionType());
   }
   else if(name == "output_filename")
   {
      ossimFilenameProperty* filenameProp =
         new ossimFilenameProperty(name, theOutputName);
      
      filenameProp->setIoType(ossimFilenameProperty::ossimFilenamePropertyIoType_OUTPUT);
      
      return filenameProp;
   }
   else if(name == "tile_size")
   {
      return new ossimStringProperty(name, getTileSize());
   }
   else if(name == "sift_nfeatures")
   {
	   return new ossimStringProperty(name, ossimString::toString(getSiftNfeatures()));
   }
   else if(name == "sift_noctavelayers")
   {
	   return new ossimStringProperty(name, ossimString::toString(getSiftNOctaveLayers()));
   }
   else if(name == "sift_contrastthreshold")
   {
	   return new ossimStringProperty(name, ossimString::toString(getSiftContrastThreshold()));
   }
   else if(name == "sift_edgethreshold")
   {
	   return new ossimStringProperty(name, ossimString::toString(getSiftEdgeThreshold()));
   }
   else if(name == "sift_sigma")
   {
	   return new ossimStringProperty(name, ossimString::toString(getSiftSigma()));
   }
   else
   {
      return ossimOutputSource::getProperty(name);
   }

   return result;
}

void ossimImageCorrelator::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
   
   propertyNames.push_back("master_filename");
   propertyNames.push_back("slave_filename");
   propertyNames.push_back("projection_filename");
   propertyNames.push_back("master_band");
   propertyNames.push_back("slave_band");
   propertyNames.push_back("scale_ratio");
   propertyNames.push_back("point_number");
   propertyNames.push_back("slave_accuracy");
   propertyNames.push_back("projection_type");
   propertyNames.push_back("output_filename");
   propertyNames.push_back("tile_size");

   propertyNames.push_back("sift_nfeatures");
   propertyNames.push_back("sift_noctavelayers");
   propertyNames.push_back("sift_contrastthreshold");
   propertyNames.push_back("sift_edgethreshold");
   propertyNames.push_back("sift_sigma");
}
