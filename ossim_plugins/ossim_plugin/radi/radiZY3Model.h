#ifndef radiZY3Model_HEADER
#define radiZY3Model_HEADER
#include <iostream>
using namespace std;
#include <ossim/projection/ossimSensorModel.h>
#include <ossim/base/ossimIpt.h>
#include <ossim/base/ossimFilename.h>
#include <ossim/base/ossimGpt.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/base/ossimEcefRay.h>
#include <ossim/base/ossimEcefPoint.h>
#include <ossim/base/ossimMatrix3x3.h>
#include <ossim/base/ossimTieGptSet.h>
#include <ossim_plugin/radi/radiZY3SupportData.h>
#include <ossimPluginConstants.h>

#include <ossim/projection/ossimRpcModel.h>

namespace ossimplugins
{
class ossimQVProcSupportData;

class OSSIM_PLUGINS_DLL radiZY3Model : public ossimSensorModel
{
public:
   /*!
    * CONSTRUCTORS:
    */
   radiZY3Model();
   radiZY3Model(ossimQVProcSupportData* sd);
   radiZY3Model(radiZY3SupportData* sd);
   radiZY3Model(const ossimFilename& init_file);
   radiZY3Model(const ossimKeywordlist& geom_kwl);
   radiZY3Model(const radiZY3Model& rhs);
   virtual ~radiZY3Model();
   enum AdjustParamIndex
   {
      CCD_ROLL_OFFSET = 0,
      CCD_PITCH_OFFSET,
      CCD_YAW_OFFSET,
	  X_OFFSET,
	  Y_OFFSET,
	  Z_OFFSET,
	  FOCAL_OFFSET,
	  CCD_OFFSET,
	  AX0_OFFSET,
	  AX1_OFFSET,
	  AX2_OFFSET,
	  AX3_OFFSET,
	  AY0_OFFSET,
	  AY1_OFFSET,
	  AY2_OFFSET,
	  AY3_OFFSET,
	  AY4_OFFSET,
	  AY5_OFFSET,
	  AY6_OFFSET,
	  AZ0_OFFSET,
	  AZ1_OFFSET,
	  AZ2_OFFSET,
	  AZ3_OFFSET,
	  LINE_OFFSET,
	  CCD_ROT_CENTER_OFFSET,
	  CCD_ROT_ANGLE,
	  CCD_CENTER_X_OFFSET,
	  CCD_CENTER_Y_OFFSET,
	  CCD_K1,
	  CCD_K2,
	  CCD_P1,
	  CCD_P2,
      NUM_ADJUSTABLE_PARAMS // not an index
   };   
   /*!
    * Returns pointer to a new instance, copy of this.
    * Not implemented yet!  Returns NULL...
    */
   virtual ossimObject* dup() const;
   /*!
    * Extends base-class implementation. Dumps contents of object to ostream.
    */
   virtual std::ostream& print(std::ostream& out) const;
   /*!
    * Fulfills ossimObject base-class pure virtuals. Loads and saves geometry
    * KWL files. Returns true if successful.
    */
   virtual bool saveState(ossimKeywordlist& kwl,
                          const char* prefix=NULL) const;
   
   virtual bool loadState(const ossimKeywordlist& kwl,
                          const char* prefix=NULL);
   /*!
    * Writes a template of geom keywords processed by loadState and saveState
    * to output stream.
    */
   static void writeGeomTemplate(ostream& os);
   /*!
    * Given an image point and height, initializes worldPoint.
    */
   virtual void lineSampleHeightToWorld(const ossimDpt& image_point,
                                        const ossim_float64& heightEllipsoid,
                                        ossimGpt& worldPoint) const;
   /*!
    * Given an image point, returns a ray originating at some arbitrarily high
    * point (ideally at the sensor position) and pointing towards the target.
	*/
   virtual void  lineSampleToWorld(const ossimDpt& image_point,
	   ossimGpt&       world_point) const;
   virtual void  worldToLineSample(const ossimGpt& world_point,
	   ossimDpt&       image_point) const;
   virtual void imagingRay(const ossimDpt& image_point,
                           ossimEcefRay&   image_ray) const;
   /*!
    * Following a change to the adjustable parameter set, this virtual
    * is called to permit instances to compute derived quantities after
    * parameter change.
    */
   virtual void updateModel();
   /*!
    * ossimOptimizableProjection
    */
   inline virtual bool useForward()const {return true;/*false;*/} //!image to ground faster
   virtual bool setupOptimizer(const ossimString& init_file); //!uses file path to init model
   bool initFromMetadata(ossimQVProcSupportData* sd);
   void setImageRect(const ossimDrect& roi);
   ossimDrect getImageClipRect()const {return theImageClipRect;};
   //void buildNormalEquation(const ossimTieGptSet& tieSet,
	  // NEWMAT::SymmetricMatrix& A,
	  // NEWMAT::ColumnVector& residue,
	  // NEWMAT::ColumnVector& projResidue,
	  // double pstep_scale);
   //ossimDpt getForwardDeriv(int iAtt, int iComponent, const ossimGpt& gpos, double hdelta=1e-11);
   //virtual double optimizeFit(const ossimTieGptSet& tieSet,
   // double* targetVariance=0);
   ossimEcefPoint collinearEquation_inverse(const ossimDpt& image_point,
	   const ossim_float64& ecef_Z,
	   const NEWMAT::Matrix& rotationMat,
	   const ossimEcefPoint& satPoint) const;
   ossimDpt collinearEquation_forward(const ossimEcefPoint& ecefPoint,
	   const NEWMAT::Matrix& rotationMat,
	   const ossimEcefPoint& satPoint) const;
   void collinearEquation_deriv(const ossimEcefPoint& ecefPoint,
	   const NEWMAT::Matrix& rotationMat,
	   const ossimEcefPoint& satPoint,
	   ossimDpt& dX, 
	   ossimDpt& dY,
	   ossimDpt& dZ) const;
   void collinearEquation_lineSampleToWorld(const ossimDpt& image_point,
	   ossimGpt& worldPoint) const;
   double calcGAST(double t) const;
   void getExteriorParameters(const ossimDpt& image_point, 
	   NEWMAT::Matrix& rotationMat,
	   ossimEcefPoint& satPoint) const;

   ossimRefPtr<ossimRpcModel>  theApproximateModel;
   void updateApproximateModel();
   void create3DGridPoints(const ossimDrect& imageBounds,
	   double height,
	   ossimTieGptSet*& pTieGptSet,
	   ossim_uint32 xSamples = 8,
	   ossim_uint32 ySamples = 8,
	   bool latlon = false,
	   bool shiftTo0Flag = false);

   ossim_float64  theRollOffset;
   ossim_float64  thePitchOffset;
   ossim_float64  theYawOffset;
   ossimRefPtr<ossimQVProcSupportData> theSupportData;
   ossim_float64  getFocalOffset(){ return theFocalOffset;}
   ossim_float64  getCcdRoolOffset(){return theCcdRollOffset;}
   ossim_float64  getCcdPitchOffset(){return theCcdPitchOffset;}
   ossim_float64  getCcdYawOffset(){return theCcdYawOffset;}
   ossim_float64  getAY0Offset(){return theAY0Offset;}
   ossim_float64  getAY1Offset(){return theAY1Offset;}
   ossim_float64  getAY2Offset(){return theAY2Offset;} 
   ossim_float64  getAY3Offset(){return theAY3Offset;} 
   ossim_float64  getAY4Offset(){return theAY4Offset;} 
   ossim_float64  getAY5Offset(){return theAY5Offset;} 
   ossim_float64  getAY6Offset(){return theAY6Offset;} 

   void  setFocalOffset(ossim_float64 a){ theFocalOffset = a;}
   void  setCcdRoolOffset(ossim_float64 a){theCcdRollOffset = a;}
   void  setCcdPitchOffset(ossim_float64 a){theCcdPitchOffset = a;}       // degrees
   void  setCcdYawOffset(ossim_float64 a){theCcdYawOffset = a;}         // degrees
   void  setAY0Offset(ossim_float64 a){theAY0Offset = a;}
   void  setAY1Offset(ossim_float64 a){theAY1Offset = a;}
   void  setAY2Offset(ossim_float64 a){theAY2Offset = a;} 
   void  setAY3Offset(ossim_float64 a){theAY3Offset = a;} 
   void  setAY4Offset(ossim_float64 a){theAY4Offset = a;} 
   void  setAY5Offset(ossim_float64 a){theAY5Offset = a;} 
   void  setAY6Offset(ossim_float64 a){theAY6Offset = a;} 
   ossim_float64 computePhiX(double samp) const;
   ossim_float64 computePhiY(double samp) const;
   void computePhiXY(double samp, double &phiX, double& phiY) const;
   void imagingRay(const ossimDpt& image_point,
	   ossimColumnVector3d& image_ray) const;
protected:
   /*!
    * Sets adjustables to default values.
    */
   void initAdjustableParameters();
   
   void loadGeometry(FILE*);
   void loadSupportData();
   void getCcdToSatRotation(NEWMAT::Matrix& result)const;
   void computeSatToOrbRotation(NEWMAT::Matrix& result, ossim_float64 t)const;
   ossim_float64 computeLookAngleY(double samp, double ccdOffset = 0.0)const;
/*    virtual ossimDpt extrapolate (const ossimGpt& gp) const; */
/*    virtual ossimGpt extrapolate (const ossimDpt& ip, */
/* 				 const double& height=ossim::nan()) const; */
   ossimFilename  theMetaDataFile;
   ossim_float64  theIllumAzimuth;  
   ossim_float64  theIllumElevation;
   ossim_float64  thePositionError;
   ossim_float64 theFocalLen;	// meters
   ossim_float64 theCcdStepLen;	// meters
   ossim_float64  theLineSamplingPeriod;
   ossim_float64  theRefImagingTime;
   ossim_float64  theRefImagingTimeLine;
   ossimDpt       theSpotSubImageOffset;
   ossim_float64  theCcdRollOffset;        // degrees
   ossim_float64  theCcdPitchOffset;       // degrees
   ossim_float64  theCcdYawOffset;         // degrees
   ossim_float64  theXOffset;
   ossim_float64  theYOffset;
   ossim_float64  theZOffset;
   ossim_float64  theFocalOffset;
   ossim_float64  theCcdOffset;
   ossim_float64  theAX0Offset;
   ossim_float64  theAX1Offset;
   ossim_float64  theAX2Offset; 
   ossim_float64  theAX3Offset; 
   ossim_float64  theAY0Offset;
   ossim_float64  theAY1Offset;
   ossim_float64  theAY2Offset; 
   ossim_float64  theAY3Offset; 
   ossim_float64  theAY4Offset; 
   ossim_float64  theAY5Offset; 
   ossim_float64  theAY6Offset; 
   ossim_float64  theAZ0Offset;
   ossim_float64  theAZ1Offset;
   ossim_float64  theAZ2Offset; 
   ossim_float64  theAZ3Offset; 
   ossim_float64  theLineOffset;
   ossim_float64  theCcdRotCenterOffset;
   ossim_float64  theCcdRotAngle;
   ossim_float64  theCcdCenterXOffset;
   ossim_float64  theCcdCenterYOffset;
   ossim_float64  theCcdK1;
   ossim_float64  theCcdK2;
   ossim_float64  theCcdP1;
   ossim_float64  theCcdP2;
   
   //NEWMAT::Matrix theCcdToSatRotation;
TYPE_DATA
};
}
#endif /* #ifndef radiZY3Model_HEADER */
