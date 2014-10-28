// ossimImageCorrelator
// class for getting tie-points from a master/slave image pair
//
// TODO : generate one file only : XML or Tabulated Text
// TODO : change TieGPtSet to a generic TiePtSet
// TODO : increase speed

#ifndef ossimImageCorrelator_HEADER
#define ossimImageCorrelator_HEADER

#include <ossim/base/ossimString.h>
#include <ossim/base/ossimFilename.h>
#include <ossim/base/ossimOutputSource.h>
#include <ossim/base/ossimProcessInterface.h>
#include <ossim/base/ossimProcessProgressEvent.h>
#include <ossim/imaging/ossimFilterResampler.h>
#include <ossim/imaging/ossimImageChain.h>
#include <ossim/imaging/ossimImageHandler.h>
#include <ossim/imaging/ossimBandSelector.h>
#include <ossim/imaging/ossimImageRenderer.h>
#include <ossim/imaging/ossimCastTileSourceFilter.h>
#include <ossim/base/ossimTDpt.h>
#include <ossim/base/ossimTieGptSet.h>
#include "ossimRegistrationExports.h"
#include <vector>
//#include <mpi.h>
//#include <ossim/parallel/ossimMpi.h>

#include <ossim/elevation/ossimElevManager.h>

#include <ogrsf_frmts.h>
#include <gdal.h>
#include <ogr_api.h>
#include <ogr_geometry.h>
#include <ogr_feature.h>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <algorithm>
#include "RANSAC/ransac/ransac.h"
#include "RANSAC/estimators/Solver.h"
#include "RANSAC/estimators/affineSolver.h"
#include "RANSAC/estimators/affineError.h"
#include "GdalRasterApp.h"
using namespace groupsac;
using namespace std;
using namespace cv;

class ossimImageGeometry;
class ossimMapProjection;
class ossimListenerManager;

class OSSIM_REGISTRATION_DLL ossimImageCorrelator :
    public ossimOutputSource,
    public ossimProcessInterface
{
public:
   ossimImageCorrelator();
   virtual ~ossimImageCorrelator();

   enum match_state{
	   success = 0,
	   slave_faild,
	   master_faild,
	   match_failed,
   };

   //accessors to parms
   inline void               setMaster(const ossimFilename& m) { theMaster=m; }
   inline const ossimFilename& getMaster()const { return theMaster; }
   inline void               setSlave(const ossimFilename& s) { theSlave=s; }
   inline const ossimFilename& getSlave()const { return theSlave; }
   inline void               setProjectionFile(const ossimFilename& s) { theProjectionFile=s; }
   inline const ossimFilename& getProjectionFile()const { return theProjectionFile; }
   inline void               setMasterBand(ossim_uint32 b) { theMasterBand=b; }
   inline ossim_uint32       getMasterBand()const { return theMasterBand; }
   inline void               setSlaveBand(ossim_uint32 b) { theSlaveBand=b; }
   inline ossim_uint32       getSlaveBand()const { return theSlaveBand; }
   inline void               setScaleRatio(const ossim_float64& r) { theScaleRatio=r; }
   inline ossim_float64      getScaleRatio()const { return theScaleRatio; }
   inline void               setSlaveAccuracy(const ossim_float64& a) { theSlaveAccuracy=a; }
   inline ossim_float64      getSlaveAccuracy()const { return theSlaveAccuracy; }
   inline void               setProjectionType(const ossimString& p) { theProjectionType=p; }
   inline const ossimString& getProjectionType()const { return theProjectionType; }
   inline void               setMasterPointProj(const ossimString& p) { theMasterPointProj=p; }
   inline const ossimString& getMasterPointProj()const { return theMasterPointProj; }
   inline void               setSlavePointProj(const ossimString& p) { theSlavePointProj=p; }
   inline const ossimString& getSlavePointProj()const { return theSlavePointProj; }
   inline void               setPointNumber(const ossim_uint32& n) { thePointNumber = n; }
   inline ossim_uint32       getPointNumber()const { return thePointNumber; }
   inline void               setTileSize(const ossim_float64& n) { theTileSize = n; }
   inline ossim_float64      getTileSize()const { return theTileSize; }

   inline void               setSiftNfeatures(const int& n) { theSiftNfeatures=n; }
   inline int      getSiftNfeatures()const { return theSiftNfeatures; }   
   inline void               setSiftNOctaveLayers(const int& a) { theSiftNOctaveLayers=a; }
   inline int      getSiftNOctaveLayers()const { return theSiftNOctaveLayers; }
   inline ossim_float64       getSiftContrastThreshold()const { return theSiftContrastThreshold; }
   inline void               setSiftContrastThreshold(const ossim_float64& r) { theSiftContrastThreshold = r; }
   inline ossim_float64       getSiftEdgeThreshold()const { return theSiftEdgeThreshold; }
   inline void               setSiftEdgeThreshold(const ossim_float64& r) { theSiftEdgeThreshold = r; }
   inline ossim_float64       getSiftSigma()const { return theSiftSigma; }
   inline void               setSiftSigma(const ossim_float64& r) { theSiftSigma = r; }
   
   inline bool hasRun()const { return theHasRun; }

   // inherited methods
   virtual bool isOpen() const;
   virtual bool open();
   virtual void close();

   virtual bool  execute(); //also creates tie point file
   virtual       ossimObject* getObject()      { return this; }
   virtual const ossimObject* getObject()const { return this; }
   virtual       ossimObject* getObjectInterface() { return this; }
   
   virtual bool canConnectMyInputTo(ossim_int32 inputIndex,const ossimConnectableObject* object)const { return false; } //TBC : so far no input

   virtual void setProperty(ossimRefPtr<ossimProperty> property);
   virtual ossimRefPtr<ossimProperty> getProperty(const ossimString& name)const;
   virtual void getPropertyNames(std::vector<ossimString>& propertyNames)const;

protected:  
	// loong
	void writeTiePoints(const ossimTieGptSet& tp);
	bool getMasterList(ossimFilename spatial_index_file, vector<ossimFilename>& masterList,
		ossimGpt ul, ossimGpt lr);
	void setAreaOfInterest(const ossimIrect& rect);
	void setOutputName(const ossimString& filename);
	inline bool getStoreFlag()const   { return theStoreFlag; }
	inline void setStoreFlag(bool sf) { theStoreFlag = sf; }
	bool getAllFeatures();
	bool getGridFeatures(const ossimIrect& rect);
	int runMatch(const ossimIrect &srect, const ossimIrect &mrect, vector<ossimTDpt>& theTies, ossim_uint32 resLevel = 0);
	static bool getGridFeaturesParallel(const ossimIrect& rect, void *pData);
	bool getGridFeaturesParallel(const ossimIrect& rect);
	static int runMatchParallel(const cv::Mat& slaveMat, const cv::Mat& masterMat, ossimTDpt& tDpt, void *pData, bool bDebug = false);
	static bool createTileMat(const ossimRefPtr<ossimCastTileSourceFilter>& cast, const ossimIrect& rect, cv::Mat& outMat, ossim_uint32 resLevel = 0);
	static ossimIpt slave2master(ossimProjection* slaveProjection,
		ossimProjection* masterProjection,
		ossimIpt slaveDpt);
	static ossimDpt slave2master(ossimProjection* slaveProjection,
		ossimProjection* masterProjection,
		ossimDpt slaveDpt);
	ossimDpt slave2master(ossimDpt slaveDpt);
	ossimIpt slave2master(ossimIpt slaveDpt);
   ossimString         getRole() const;
   ossimImageHandler*  getProjectionHandler();
   
   ossimRefPtr<ossimImageGeometry> getOutputImageGeometry();

   ossimMapProjection* getOutputProjection();
   
   bool buildRenderer(
      ossimImageChain* chain,
//         ossimImageSource* source, 
         ossimMapProjection* outProjection, 
         ossimImageRenderer* renderer,
         const ossimFilterResampler::ossimFilterResamplerType& stype =  ossimFilterResampler::ossimFilterResampler_CUBIC
         )const;

   ossimFilename     theLastMaster;
   bool              theStoreFlag;
#if OSSIM_HAS_MPI
   MPI_File          theFileStream;
#else
   fstream          theFileStream;
#endif
   ossimIrect        theAreaOfInterest;
   vector<ossimTDpt> theTiePoints;
   ossim_uint32  theMasterBand;
   ossim_uint32  theSlaveBand;
   ossim_float64 theSlaveAccuracy;
   ossimString   theProjectionType;
   ossimString   theMasterPointProj;
   ossimString   theSlavePointProj;
   ossimProjection *theMasterProjection;
   ossimProjection *theSlaveProjection;
   ossim_float64 theScaleRatio;

   ossim_uint32  thePointNumber; 	// required number
   ossimFilename   theMaster;
   ossimFilename   theSlave;
   ossimFilename	  theFilename;
   ossimFilename   theProjectionFile;
   ossim_float64	  theTileSize;
   int theSiftNfeatures;
   int theSiftNOctaveLayers;
   double theSiftContrastThreshold;
   double theSiftEdgeThreshold;
   double theSiftSigma;

   bool theHasRun; //to know whether execute has been run

   ossimRefPtr<ossimImageChain> theMChain;
   ossimRefPtr<ossimImageChain> theSChain;
   
   ossimRefPtr<ossimImageHandler>   handlerM;
   ossimRefPtr<ossimImageHandler>   handlerS;
   ossimRefPtr<ossimBandSelector>   theMasterBandSelector;
   ossimRefPtr<ossimBandSelector>   theSlaveBandSelector;
   ossimRefPtr<ossimImageRenderer>  rendererM;
   ossimRefPtr<ossimImageRenderer>  rendererS;
   std::vector<ossimRefPtr<ossimCastTileSourceFilter> > caster;
   ossimTieGptSet       theTset;

   //! Disallow operator=
   const ossimImageCorrelator& operator=(const ossimImageCorrelator& rhs) {return rhs;}

TYPE_DATA
};

#endif //ossimImageCorrelator_HEADER
