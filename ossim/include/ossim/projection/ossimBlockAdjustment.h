#ifndef ossimBlockAdjustment_HEADER
#define ossimBlockAdjustment_HEADER

#include <ossim/base/ossimString.h>
#include <ossim/base/ossimTieGpt.h>
#include <ossim/base/ossimTieGptSet.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/projection/ossimSensorModel.h>
/************************************************************************/
/*                          Block Adjustment                            */
/*	Author: loong
	Date: 2010.01.20
*/
/************************************************************************/


/***************************************************************************/
/*
 *                              DEFINES
*/
typedef std::pair<ossimDpt, ossimDpt> ossimDline;
typedef std::pair<ossimGpt, ossimGpt> ossimGline;

class OSSIMDLLEXPORT ossimBlockTieGpt:public ossimTieGpt
{
public:
	ossimString m_ID;
	int m_nType;						// 0: unknown corresponding image points;
										// 1: known ground control points;
										// 2: known corresponding image points;
	vector< int > m_ImageIndices;		// begin with 0
	vector< ossimDpt > m_DptList;		// corresponding with m_ImageIndices
	int m_nUnknownIndex;				// -1:	unknown corresponding image points;
										// >-1:	known ground control points or known
										// corresponding image points (begin with 0);
public:
	/*!
	* Constructors
	*/
	ossimBlockTieGpt();
	ossimBlockTieGpt(ossimTieGpt tieGpt);
};

class OSSIMDLLEXPORT ossimBlockAdjustment
{
public:

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
		DENMARK  =8
	};
	/*!
	* CONSTRUCTORS:
	*/
	ossimBlockAdjustment();

	virtual ~ossimBlockAdjustment();

	void addSensorModel(ossimSensorModel* sensorModel);
	void adjustment(vector< ossimBlockTieGpt > GptList, RobustMode mode = NONE);
	//void adjustment(vector< ossimTieLine > TieLineList, RobustMode mode = NONE);

public:
protected:
	vector< ossimSensorModel* > m_SensorModleList;
	vector< ossimBlockTieGpt > m_GptList;
	//vector< vector< ossimBlockTieGpt >> m_GptListByImages;
	vector< int > m_FirstParaIndices;		//begin with 0
	int m_nUnknownPoints;


	NEWMAT::DiagonalMatrix m_wgtMatrix;		// weight Matrix of control data

	int initiateGpt(vector< ossimBlockTieGpt > &GptList);
	vector< vector< ossimBlockTieGpt > > classifyGptByImages(vector< ossimBlockTieGpt > GptList);
	void buildErrorEquation(const vector< vector< ossimBlockTieGpt > >& GptListByImages,
		NEWMAT::Matrix &AA, NEWMAT::Matrix &BB, NEWMAT::ColumnVector &LL,
		double pstep_scale);
	NEWMAT::ColumnVector buildNormalEquation(const vector< vector< ossimBlockTieGpt > >& GptListByImages,
																	NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
																	NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2, double pstep_scale);
	void solve2LeastSquares(NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
							NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2,
							NEWMAT::ColumnVector &deltap, NEWMAT::ColumnVector &deltac);

	NEWMAT::ColumnVector solveLeastSquares(NEWMAT::SymmetricMatrix& A,  NEWMAT::ColumnVector& r)const;
	//NEWMAT::ColumnVector solveLeastSquares(NEWMAT::Matrix& A,  NEWMAT::ColumnVector& r)const;
	NEWMAT::Matrix invert(const NEWMAT::Matrix& m)const;

	void updateSensorModels(NEWMAT::ColumnVector deltap);
	void updateCoordinates(vector< vector< ossimBlockTieGpt > > &GptListByImages, NEWMAT::ColumnVector deltac);
	NEWMAT::ColumnVector getResidue(const vector< vector< ossimBlockTieGpt > >& GptListByImages);

	void saveBlockGpt(vector<ossimBlockTieGpt> gptList, ossimString outFile, ossimMapProjection* transMerc = NULL);
	void updateWeightsMatrix(const NEWMAT::ColumnVector& newresidue, RobustMode mode = NONE);
	void updateWeightsMatrixHuber(const NEWMAT::ColumnVector& newresidue, double coeff = 2.0);
//TYPE_DATA
};

#endif /* ossimBlockAdjustment_HEADER */
