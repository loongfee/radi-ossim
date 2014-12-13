#ifndef ossimRpcRcz2XyModel_HEADER
#define ossimRpcRcz2XyModel_HEADER
#include <ossim/projection/ossimSensorModel.h>
#include <ossim/base/ossimIpt.h>
#include <ossim/base/ossimDblGrid.h>
#include <ossim/base/ossimFilename.h>

#include <alglib/optimization.h>
using namespace alglib;
/*!****************************************************************************
*
* CLASS:  ossimRpcRcz2XyModel
*
*****************************************************************************/
class OSSIMDLLEXPORT ossimRpcRcz2XyModel : public ossimSensorModel
{
public:
	enum PolynomialType
	{
		A='A',  // corresponds to "RPC00A"
		B='B'   // corresponds to "RPC00B"
	};
	/** @brief RPC model structure used for access function */
	struct rpcModelStruct
	{
		double lineScale;
		double sampScale;
		double latScale;
		double lonScale;
		double hgtScale;
		double lineOffset;
		double sampOffset;
		double latOffset;
		double lonOffset;
		double hgtOffset;
		double lineNumCoef[20];
		double lineDenCoef[20];
		double sampNumCoef[20];
		double sampDenCoef[20];
		char   type;
	};
	/** @brief default constructor */
	ossimRpcRcz2XyModel();
	/** @brief copy construtor */
	ossimRpcRcz2XyModel(const ossimRpcRcz2XyModel& copy_this);
	/** @brief virtual destructor */
	virtual ~ossimRpcRcz2XyModel();
	void setAttributes(ossim_float64 theSampleOffset,
		ossim_float64 theLineOffset,
		ossim_float64 theSampleScale,
		ossim_float64 theLineScale,
		ossim_float64 theLatOffset,
		ossim_float64 theLonOffset,
		ossim_float64 theHeightOffset,
		ossim_float64 theLatScale,
		ossim_float64 theLonScale,
		ossim_float64 theHeightScale,
		const std::vector<double>& xNumeratorCoeffs,
		const std::vector<double>& xDenominatorCoeffs,
		const std::vector<double>& yNumeratorCoeffs,
		const std::vector<double>& yDenominatorCoeffs,
		PolynomialType polyType = B,
		bool computeGsdFlag=true);
	void setAttributes(ossimRpcRcz2XyModel::rpcModelStruct& model, bool computeGsdFlag=true);
	void setMetersPerPixel(const ossimDpt& metersPerPixel);
	/**
	* This method computes the ground sample distance(gsd) and sets class
	* attribute theGSD and theMeanGSD by doing a lineSampleHeightToWorld on
	* three points and calculating the distance from them.
	*
	* @return Nothing but throws ossimException on error.
	*/
	void computeGsd();
	/**
	* @brief Sets data member theBiasError, theRandError.
	*
	* @param biasError Error - Bias 68% non time - varying error estimate
	* assumes correlated images.  Units = meters.
	*
	* @param randError Error - Random 68% time - varying error estimate
	* assumes uncorrelated images. Units = meters.
	*
	* @param initNominalPostionErrorFlag If true the base data member
	* theNominalPosError will be initialized with:
	* sqrt(theBiasError*theBiasError +theRandError*theRandError)
	*/
	void setPositionError(const ossim_float64& biasError,
		const ossim_float64& randomError,
		bool initNominalPostionErrorFlag);
	/**
	* @brief worldToLineSample()
	* Overrides base class implementation. Directly computes line-sample from
	* the polynomials.
	*/
	virtual void  worldToLineSample(const ossimGpt& world_point,
		ossimDpt&       image_point) const;
	/**
	* @brief print()
	* Extends base-class implementation. Dumps contents of object to ostream.
	*/
	virtual std::ostream& print(std::ostream& out) const;
	/**
	* @brief saveState
	* Fulfills ossimObject base-class pure virtuals. Loads and saves geometry
	* KWL files. Returns true if successful.
	*/
	virtual bool saveState(ossimKeywordlist& kwl,
		const char* prefix=0) const;

	/**
	* @brief loadState
	* Fulfills ossimObject base-class pure virtuals. Loads and saves geometry
	* KWL files. Returns true if successful.
	*/
	virtual bool loadState(const ossimKeywordlist& kwl,
		const char* prefix=0);
	virtual void  lineSampleToWorld(const ossimDpt& image_point,
		ossimGpt&       world_point) const;
	virtual void lineSampleHeightToWorld(const ossimDpt& image_point,
		const double&   heightEllipsoid,
		ossimGpt&       worldPoint) const;

	/**
	* @brief imagingRay()
	* Overrides base class pure virtual.
	*/
	virtual void imagingRay(const ossimDpt& image_point,
		ossimEcefRay&   image_ray) const;
	/**
	* @brief STATIC METHOD: writeGeomTemplate(ostream)
	* Writes a template of geom keywords processed by loadState and saveState
	* to output stream.
	*/
	static void writeGeomTemplate(ostream& os);
	virtual void updateModel();
	virtual void initAdjustableParameters();
	/**
	* @brief dup()
	* Returns pointer to a new instance, copy of this.
	*/
	virtual ossimObject* dup() const;
	inline virtual bool useForward()const {return false;}
	/** @brief uses file path to init model */
	virtual bool setupOptimizer(const ossimString& init_file);
	/**
	* @brief Compute partials of samp/line WRT ground point
	*
	* @param parmIdx computational mode:
	*        OBS_INIT, EVALUATE, P_WRT_X, P_WRT_X, P_WRT_X.
	*
	* @param gpos Current ground point estimate.
	*
	* @param h Not used.
	*
	* @return OBS_INT: n/a, EVALUATE: residuals, P_WRT_X/Y/Z: partials.
	*/
	virtual ossimDpt getForwardDeriv(int parmIdx,
		const ossimGpt& gpos,
		double h);
	/**
	* @brief Returns Error - Bias.
	* @return Error - Bias
	* @note See NITF field "ERR_BIAS" from RPC00x tag where x = A or B.
	*/
	double getBiasError() const;
	/**
	* @brief Returns Error - Random.
	* @return Error - Random
	* @note See NITF field "ERR_RAND" from RPC00x tag where x = A or B.
	*/
	double getRandError() const;
	/**
	* @brief Returns RPC parameter set in structure.
	* @param rpcModelStruct structure to initialize.
	*/
	void getRpcParameters(ossimRpcRcz2XyModel::rpcModelStruct& model) const;


	// loong 2012.2
	vector<ossimDpt> getModelForwardDeriv(const ossimGpt& pos, double h);
	void updateRpcModelParams(NEWMAT::ColumnVector deltap);

	int getModelParamNum()const {return 80;};

	enum rpcOptimizeType
	{
		allParam,		//所有参数
		imageAffine,	//像方仿射变换
		unknown,		//未知
	};
	rpcOptimizeType m_optimizeType;	
	void saveRpcModelStruct(fstream &fs)const;
	static void rpcForward(int mode, int n, const NEWMAT::ColumnVector& x, double& fx, NEWMAT::ColumnVector& g, int& result, const NEWMAT::ColumnVector& paramList);
	static void init_rpcForward(int n, NEWMAT::ColumnVector& x, const NEWMAT::ColumnVector& paramList);


protected:
	enum AdjustParamIndex
	{
		INTRACK_OFFSET = 0,
		CRTRACK_OFFSET,
		INTRACK_SCALE,
		CRTRACK_SCALE,
		MAP_ROTATION,
		NUM_ADJUSTABLE_PARAMS // not an index
	};
	double polynomial(const double& nline,
		const double& nsamp,
		const double& nhgt,
		const double* coeffs) const;
	double dPoly_dLine(const double& nline,
		const double& nsamp,
		const double& nhgt,
		const double* coeffs) const;
	double dPoly_dSamp(const double& nline,
		const double& nsamp,
		const double& nhgt,
		const double* coeffs) const;
	double dPoly_dHgt(const double& nline,
		const double& nsamp,
		const double& nhgt,
		const double* coeffs) const;

	PolynomialType thePolyType;
	double theLineScale;
	double theSampScale;
	double theLatScale;
	double theLonScale;
	double theHgtScale;
	double theLineOffset;
	double theSampOffset;
	double theLatOffset;
	double theLonOffset;
	double theHgtOffset;

	double theIntrackOffset;
	double theCrtrackOffset;
	double theIntrackScale;
	double theCrtrackScale;
	double theCosMapRot;
	double theSinMapRot;
	/** error */
	double theBiasError; // meters
	double theRandError; // meters
	double theLineNumCoef[20];
	double theLineDenCoef[20];
	double theSampNumCoef[20];
	double theSampDenCoef[20];

	TYPE_DATA
};
#endif
