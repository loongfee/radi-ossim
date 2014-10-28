#ifndef radiBlockAdjustment_HEADER
#define radiBlockAdjustment_HEADER

#include <ossim/base/ossimString.h>
#include <ossim/base/ossimTieGpt.h>
#include <ossim/base/ossimTieGptSet.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/projection/ossimSensorModel.h>
#include <ossim_plugin/radi/radiBlockTieGpt.h>
#include <ossim_plugin/radi/radiBlockTieGptSet.h>
#include <ossim/elevation/ossimElevManager.h>
#include <ossimPluginConstants.h>
//#include <dogleg.h>
#include <suitesparse/cholmod.h>
#include <splm.h>

namespace ossimplugins
{
/************************************************************************/
/*                          Block Adjustment                            */
/*	Author: loong
	Date: 2010.01.20
	Modified on 2014.08.21
*/
/************************************************************************/


class OSSIM_PLUGINS_DLL radiBlockAdjustment
{
public:

	struct POS_RANGE 
	{
		int begin;
		int end;
	};

	enum SensorType
	{
		modelLandsat5,
		modelSpot5,
		modelLandsat7,
		modelTheos,
		modelAlos,
		modelAlosAVNIR2_1B2,
		modelAlosPRISM_1B2,
		UnknowMole
	};

	enum RobustMode
	{
		NONE	 =0,
		MEDIAN   =1,
		TURKEY   =2,
		HUBER    =3,
		HAMPEL   =4,
		Anddrews =5,
		IGG1     =6,
		FAIR     =7,
		DENMARK  =8,
		BISQUARE,
	};

	struct optimzeDataStruct
	{
		radiBlockAdjustment* pBa;
		std::map<int, radiBlockTieGptSet>* GptMapByImages;
		RobustMode mode;
	};
	/*!
	* CONSTRUCTORS:
	*/
	radiBlockAdjustment();

	virtual ~radiBlockAdjustment();

	void addSensorModel(int id, ossimSensorModel* sensorModel);
	void adjustment(vector< radiBlockTieGpt > GptList, RobustMode mode = NONE);
	void adjustment(radiBlockTieGptSet& GptList, RobustMode mode = NONE);
	//void dogleg_adjustment(radiBlockTieGptSet& GptList, RobustMode mode = NONE);
	void sparselm_adjustment(radiBlockTieGptSet& GptList, RobustMode mode = NONE);
	//void adjustment(vector< ossimTieLine > TieLineList, RobustMode mode = NONE);

	void lineSampleToWorld(pair<int, ossimDpt> imagePt, ossimGpt& gpt) const;
	//static void optimizerCallback(const double* p,
	//	double* x,
	//	cholmod_sparse* Jt,
	//	void* cookie );

	static void splm_func(double *p, double *hx, int m, int n, void *adata);
	static void splm_jac(double *p, struct splm_crsm *jac, int m, int n, void *adata);

public:
protected:
	std::map<int, ossimSensorModel*> m_SensorModleMap;
	vector< radiBlockTieGpt > m_GptList;
	//vector< vector< ossimBlockTieGpt >> m_GptListByImages;
	std::map< int, POS_RANGE> m_ParamPosMap;		//begin with 0 {imageId, value}
	int m_nUnknownPoints;

	NEWMAT::DiagonalMatrix m_wgtMatrix;		// weight Matrix of control data

	int initiateGpt(vector< radiBlockTieGpt > &GptList);
	int initiateGpt(radiBlockTieGptSet &GptList);
	vector< vector< radiBlockTieGpt > > classifyGptByImages(vector< radiBlockTieGpt > GptList);
	std::map<int, radiBlockTieGptSet > classifyGptByImages(radiBlockTieGptSet GptList);
	void buildErrorEquation(const vector< vector< radiBlockTieGpt > >& GptListByImages,
		NEWMAT::Matrix &AA, NEWMAT::Matrix &BB, NEWMAT::ColumnVector &LL,
		double pstep_scale);
	void buildErrorEquation(const std::map<int, radiBlockTieGptSet>& GptMapByImages,
		NEWMAT::Matrix &AA, NEWMAT::Matrix &BB, NEWMAT::ColumnVector &LL,
		double pstep_scale);
	NEWMAT::ColumnVector buildNormalEquation(const vector< vector< radiBlockTieGpt > >& GptListByImages,
																	NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
																	NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2, double pstep_scale);
	NEWMAT::ColumnVector buildNormalEquation(const std::map<int, radiBlockTieGptSet>& GptMapByImages, 
		NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
		NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2, double pstep_scale);
	//void solve2LeastSquares(NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
	//						NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2,
	//						NEWMAT::ColumnVector &deltap, NEWMAT::ColumnVector &deltac);

	//NEWMAT::ColumnVector solveLeastSquares(NEWMAT::SymmetricMatrix& A,  NEWMAT::ColumnVector& r)const;
	////NEWMAT::ColumnVector solveLeastSquares(NEWMAT::Matrix& A,  NEWMAT::ColumnVector& r)const;
	//NEWMAT::Matrix invert(const NEWMAT::Matrix& m)const;

	void updateSensorModels(NEWMAT::ColumnVector deltap);
	void updateCoordinates(vector< vector< radiBlockTieGpt > > &GptListByImages, NEWMAT::ColumnVector deltac);
	void updateCoordinates(std::map<int, radiBlockTieGptSet>& GptMapByImages, NEWMAT::ColumnVector deltac);
	NEWMAT::ColumnVector getResidue(const vector< vector< radiBlockTieGpt > >& GptListByImages);
	NEWMAT::ColumnVector getResidue(const std::map<int, radiBlockTieGptSet>& GptMapByImages);

	void saveBlockGpt(vector<radiBlockTieGpt> gptList, ossimString outFile, ossimMapProjection* transMerc = NULL);
	void updateWeightsMatrix(const NEWMAT::ColumnVector& newresidue, RobustMode mode = NONE);
	void updateWeightsMatrixHuber(const NEWMAT::ColumnVector& newresidue, double k = 1.345);
	void updateWeightsMatrixBisquare(const NEWMAT::ColumnVector& newresidue, double k = 4.685);
//TYPE_DATA
};
} // end of namespace ossimplugins

#endif /* radiBlockAdjustment_HEADER */
