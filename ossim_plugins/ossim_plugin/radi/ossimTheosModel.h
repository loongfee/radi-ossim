#ifndef ossimTheosModel_HEADER
#define ossimTheosModel_HEADER
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
class ossimTheosDimapSupportData;
class OSSIMDLLEXPORT ossimTheosModel : public ossimSensorModel
{
public:
   /*!
    * CONSTRUCTORS:
    */
   ossimTheosModel();
   ossimTheosModel(ossimTheosDimapSupportData* sd);
   ossimTheosModel(const ossimFilename& init_file);
   ossimTheosModel(const ossimKeywordlist& geom_kwl);
   ossimTheosModel(const ossimTheosModel& rhs);
   virtual ~ossimTheosModel();
   enum AdjustParamIndex
   {
      ROLL_OFFSET = 0,
      PITCH_OFFSET,
      YAW_OFFSET,
      ROLL_RATE,
      PITCH_RATE,
      YAW_RATE,
      FOCAL_LEN_OFFSET,
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
   bool initFromMetadata(ossimTheosDimapSupportData* sd);
protected:
   /*!
    * Sets adjustables to default values.
    */
   void initAdjustableParameters();
   
   void loadGeometry(FILE*);
   void loadSupportData();
   void computeSatToOrbRotation(NEWMAT::Matrix& result, ossim_float64 t)const;
/*    virtual ossimDpt extrapolate (const ossimGpt& gp) const; */
/*    virtual ossimGpt extrapolate (const ossimDpt& ip, */
/* 				 const double& height=ossim::nan()) const; */
   ossimRefPtr<ossimTheosDimapSupportData> theSupportData;
   ossimFilename  theMetaDataFile;
   ossim_float64  theIllumAzimuth;  
   ossim_float64  theIllumElevation;
   ossim_float64  thePositionError;
   ossim_float64  theRefImagingTime;
   /** relative to full image */
   ossim_float64  theRefImagingTimeLine;
   
   ossim_float64  theLineSamplingPeriod;
   ossimDpt       theTheosSubImageOffset;
   ossim_float64  theRollOffset;      // degrees
   ossim_float64  thePitchOffset;     // degrees
   ossim_float64  theYawOffset;       // degrees
   ossim_float64  theRollRate;        // degrees/sec
   ossim_float64  thePitchRate;       // degrees/sec
   ossim_float64  theYawRate;         // degrees/sec
   ossim_float64  theFocalLenOffset;  // percent deviation from nominal

   const ossimDatum *aDatumWant;
TYPE_DATA
};
#endif /* #ifndef ossimTheosModel_HEADER */
