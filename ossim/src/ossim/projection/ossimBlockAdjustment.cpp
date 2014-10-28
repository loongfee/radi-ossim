#include <cstdlib>
#include <ossim/projection/ossimBlockAdjustment.h>

#pragma warning(push)
#pragma warning(disable : 4482)
ossimBlockTieGpt::ossimBlockTieGpt()
:ossimTieGpt()
{

}
ossimBlockTieGpt::ossimBlockTieGpt(ossimTieGpt tieGpt)
:ossimTieGpt(tieGpt)
{
	//*this = dynamic_cast(tieGpt);
}

ossimBlockAdjustment::ossimBlockAdjustment()
{
   
}

ossimBlockAdjustment::~ossimBlockAdjustment()
{
	
}

void ossimBlockAdjustment::addSensorModel(ossimSensorModel* sensorModel)
{
	int npara = sensorModel->getNumberOfAdjustableParameters();
	if(m_FirstParaIndices.size() == 0)
	{
		m_FirstParaIndices.push_back(0);
		m_FirstParaIndices.push_back(npara);
	}
	else
		m_FirstParaIndices.push_back(m_FirstParaIndices.back() + npara);
	m_SensorModleList.push_back(sensorModel);
}

// give the unknown corresponding image points initial values
int ossimBlockAdjustment::initiateGpt(vector< ossimBlockTieGpt > &GptList)
{
	int nPoints = GptList.size();
	ossimGpt gpt;

	m_nUnknownPoints = 0;
	for(int i = 0;i < nPoints;++i)
	{
		if(0 == GptList[i].m_nType)
		{// if the point is an unknown corresponding image point
			//gpt = m_SensorModleList[GptList[i].m_ImageIndices[0]]->inverse_do(GptList[i].m_DptList[0],GptList[i].getGroundPoint());
			gpt = m_SensorModleList[GptList[i].m_ImageIndices[0]]->inverse(GptList[i].m_DptList[0]);
			GptList[i].setImagePoint(GptList[i].m_DptList[0]);
			GptList[i].setGroundPoint(gpt);
			GptList[i].m_nUnknownIndex = m_nUnknownPoints++;
		}
		else
		{
			ossimDpt imagepoint,cimagepoint;
			ossimGpt goundpoint,tGpt;
			imagepoint = GptList[i].getImagePoint();
			goundpoint = GptList[i].getGroundPoint();
			tGpt = m_SensorModleList[GptList[i].m_ImageIndices[0]]->m_proj->inverse(goundpoint);
			tGpt.hgt = GptList[i].hgt;
			GptList[i].setImagePoint(GptList[i].m_DptList[0]);
			GptList[i].setGroundPoint(tGpt);
			GptList[i].m_nUnknownIndex = -1;
		}
	}
	return m_nUnknownPoints;
}

// find the gcps for each image and put them into a vector container
vector< vector< ossimBlockTieGpt >> ossimBlockAdjustment::classifyGptByImages(vector< ossimBlockTieGpt > GptList)
{
	vector< vector< ossimBlockTieGpt >> GptListByImages;
	int nImages = m_SensorModleList.size();
	for(int i = 0;i < nImages;++i)
	{
		vector< ossimBlockTieGpt > gptListTmp;
		for(int j = 0;j < static_cast<int>(GptList.size());++j)
		{
			for(int k = 0;k < static_cast<int>(GptList[j].m_ImageIndices.size());++k)
			{
				if(GptList[j].m_ImageIndices[k] == i)
				{
					ossimBlockTieGpt blockTieGpt = GptList[j];
					blockTieGpt.setImagePoint(blockTieGpt.m_DptList[k]);
					gptListTmp.push_back(blockTieGpt);
				}
			}
		}
		GptListByImages.push_back(gptListTmp);
	}
	return GptListByImages;
}

// build the block adjustment error equation
void ossimBlockAdjustment::buildErrorEquation(const vector< vector< ossimBlockTieGpt >>& GptListByImages,
											 NEWMAT::Matrix &AA, NEWMAT::Matrix &BB, NEWMAT::ColumnVector &LL,
											 double pstep_scale)
{
	int nImages = m_SensorModleList.size();
	int nTotalParam = m_FirstParaIndices.back();
	int nTotalEquation = 0;
	int nUnknownCoordinate = m_nUnknownPoints * 3;
	for(int i = 0;i < nImages;++i)
	{
		nTotalEquation += 2 * GptListByImages[i].size();
	}

	AA.ReSize(nTotalEquation, nTotalParam);
	BB.ReSize(nTotalEquation, nUnknownCoordinate);
	LL.ReSize(nTotalEquation);
	AA = 0.0;
	BB = 0.0;
	LL = 0.0;


	int nEquationIndex = 0;
	for(int i = 0;i < nImages;++i)
	{
		for(int j = 0;j < static_cast<int>(GptListByImages[i].size());++j)
		{
			NEWMAT::Matrix A;
			NEWMAT::Matrix B;
			NEWMAT::ColumnVector L;
			// use one gcp to build two error equations
			m_SensorModleList[i]->buildErrorEquation(GptListByImages[i][j], GptListByImages[i][j].m_nType, A, B, L, pstep_scale);
			
			// add the two equations to the block adjustment error equations set
			for(int k = 0;k < static_cast<int>(m_SensorModleList[i]->getNumberOfAdjustableParameters());++k)
			{
				AA.element(0 + nEquationIndex, m_FirstParaIndices[i] + k) = A.element(0, k);
				AA.element(1 + nEquationIndex, m_FirstParaIndices[i] + k) = A.element(1, k);
			}
			if(GptListByImages[i][j].m_nUnknownIndex > -1)
			{
				for(int k = 0;k < 3;++k)
				{
					BB.element(0 + nEquationIndex, GptListByImages[i][j].m_nUnknownIndex * 3 + k) = B.element(0, k);
					BB.element(1 + nEquationIndex, GptListByImages[i][j].m_nUnknownIndex * 3 + k) = B.element(1, k);
				}

			}

			LL.element(0 + nEquationIndex) = L.element(0);
			LL.element(1 + nEquationIndex) = L.element(1);

			nEquationIndex += 2;
		}
	}
}

NEWMAT::ColumnVector ossimBlockAdjustment::buildNormalEquation(const vector< vector< ossimBlockTieGpt >>& GptListByImages, 
											  NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
											  NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2, double pstep_scale)
{
	NEWMAT::Matrix A;
	NEWMAT::Matrix B;
	NEWMAT::ColumnVector L;
	buildErrorEquation(GptListByImages, A, B, L, pstep_scale);

	//ofstream fs("D:\\loong\\Programs\\BlockAdjustment\\Release\\matrix1.txt");
	//fs<<"A:"<<endl<<A<<endl<<"B:"<<endl<<B<<endl<<"L:"<<endl<<L<<endl;
	//fs.close();

	int nobs = A.Nrows();
	int np1 = A.Ncols();
	int np2 = B.Ncols();

	N11.ReSize(np1, np1);
	N12.ReSize(np1, np2);
	N22.ReSize(np2, np2);
	L1.ReSize(np1);
	L2.ReSize(np2);

	N11 = A.t() * m_wgtMatrix * A;
	N12 = A.t() * m_wgtMatrix * B;
	N22 = B.t() * m_wgtMatrix * B;
	L1 = A.t() * m_wgtMatrix * L;
	L2 = B.t() * m_wgtMatrix * L;

	return L;
}

void ossimBlockAdjustment::adjustment(vector< ossimBlockTieGpt > GptList, RobustMode mode/* = NONE*/)
{
	int nImages = m_SensorModleList.size();
	int nTotalParam = m_FirstParaIndices.back();
	int nUnknownPoints = initiateGpt(GptList);
	int nUnknownCoordinate = m_nUnknownPoints * 3;
	
	vector< vector< ossimBlockTieGpt >> GptListByImages = classifyGptByImages( GptList );

	int nTotalEquation = 0;
	for(int i = 0;i < nImages;++i)
	{
		nTotalEquation += 2 * GptListByImages[i].size();
	}

	//saveBlockGpt(GptListByImages[0], "D:\\loong\\Programs\\BlockAdjustment\\Release\\gpt1.txt", m_SensorModleList[0]->m_proj);

	//setup initail values
	int iter=0;
	int iter_max = 200;    //ww1130
	double minResidue = 1e-6; //TBC
	double minDelta = 1e-6; //TBC
	//build Least Squares initial normal equation
	// don't waste memory, add samples one at a time
	NEWMAT::Matrix N11;
	NEWMAT::Matrix N12; 
	NEWMAT::Matrix N22;
	NEWMAT::ColumnVector L1;
	NEWMAT::ColumnVector L2;
	NEWMAT::ColumnVector residue;
	NEWMAT::SymmetricMatrix A;
	NEWMAT::ColumnVector L;


	m_wgtMatrix.ReSize(nTotalEquation);
	m_wgtMatrix = 1;

	double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1

	residue = buildNormalEquation(GptListByImages, N11, N12, N22, L1, L2,deltap_scale);

	residue = getResidue(GptListByImages);
	double ki2=residue.SumSquare();

	int nobs = residue.Nrows() / 2;

	double damping_speed = 1.0;
	double maxdiag1=0.0;
	double maxdiag2=0.0;
	double damping1;
	double damping2;

	//find max diag element for N11
	maxdiag1=0.0;
	maxdiag2=0.0;
	for(int d=1;d<=N11.Nrows();++d) {if (maxdiag1 < N11(d,d)) maxdiag1 = N11(d,d);}
	for(int d=1;d<=N22.Nrows();++d) {if (maxdiag2 < N22(d,d)) maxdiag2 = N22(d,d);}
	damping1 = damping_speed * 1e-3 * maxdiag1;
	damping2 = damping_speed * 1e-3 * maxdiag2;

	double olddamping1 = 0.0;
	double olddamping2 = 0.0;

	bool found = false;

	while ( (!found) && (iter < iter_max) ) //non linear optimization loop
	{
		bool decrease = false;
		do
		{
			int n1 = N11.Nrows();
			int n2 = N22.Nrows();
			A.ReSize(n1 + n2);
			L.ReSize(n1 + n2);
			for(int i = 0;i < n1 + n2;++i)
			{
				for(int j = 0;j < n1 + n2;++j)
				{
					if(i < n1)
					{
						if(j < n1)
						{
							A.element(i,j) = N11.element(i,j);
						}
						else
						{
							A.element(i,j) = N12.element(i, j - n1);
						}
						L.element(i) = L1.element(i);
					}
					else
					{
						if(j < n1)
						{
							A.element(i,j) = N12.element(j, i - n1);
						}
						else
						{
							A.element(i,j) = N22.element(i - n1, j - n1);
						}
						L.element(i) = L2.element(i - n1);
					}
				}
			}

			//fstream fs;
			//fs.open("D:\\loong\\Programs\\BlockAdjustment\\Release\\matrix.txt",ios_base::out);
			//fs<<"N11:"<<endl<<N11<<endl<<"N12:"<<endl<<N12<<endl<<"N22:"<<endl<<N22<<endl<<"L1:"<<endl<<L1<<endl<<"L2:"<<endl<<L2<<endl;
			//fs.close();
			//fs.open("D:\\loong\\Programs\\BlockAdjustment\\Release\\matrix2.txt",ios_base::out);
			//fs<<"A:"<<endl<<A<<endl<<"L:"<<endl<<L<<endl;
			//fs.close();


			//add damping update to normal matrix
			for(int d=1;d<=N11.Nrows();++d) A(d,d) += damping1 - olddamping1;
			for(int d=1;d<=N22.Nrows();++d) A(d+N11.Nrows(),d+N11.Nrows()) += damping2 - olddamping2;
			olddamping1 = damping1;
			olddamping2 = damping2;

			NEWMAT::ColumnVector deltap;
			NEWMAT::ColumnVector deltac; //= solveLeastSquares(A, projResidue);
			deltap.ReSize(N11.Nrows());
			deltac.ReSize(N22.Nrows());
			//solve2LeastSquares(N11, N12, N22, L1, L2, deltap, deltac);


			/*fstream fs;
			fs.open("H:\\AlosData\\matrix_block.txt", ios_base::out);
			for(int i = 0;i < A.Nrows();i++)
			{
				for(int j = 0;j < A.Ncols();j++)
				{
					fs<<A.element(i,j)<<"\t";
				}
				fs<<endl;
			}
			fs<<endl<<endl;
			for(int i = 0;i < L.Nrows();i++)
			{
				fs<<L.element(i)<<endl;
			}
			fs.close();*/


			NEWMAT::ColumnVector x = solveLeastSquares(A, L);
			for(int i = 0;i < N11.Nrows();++i)
				deltap.element(i) = x.element(i);
			for(int i = 0;i < N22.Nrows();++i)
				deltac.element(i) = x.element(N11.Nrows() + i);

			cout<<x<<endl<<endl;
			if (x.NormFrobenius() <= x.Nrows() * minDelta)
			{
				found = true;
			} else {
				//update adjustment
				updateSensorModels(deltap);
				updateCoordinates(GptListByImages, deltac);

				NEWMAT::ColumnVector newresidue = getResidue(GptListByImages);
				double newki2 = newresidue.SumSquare();
				double res_reduction = (ki2 - newki2) /
					((deltap.t()*(deltap*damping1 + L1)).AsScalar()
					+ (deltac.t()*(deltac*damping2 + L2)).AsScalar());

				//DEBUG TBR
				cout<<sqrt(newki2/nobs)<<" the "<<iter<<"th iteration"<<endl;
				cout.flush();
				if (res_reduction > 0)
				{
					//accept new parms
					ki2=newki2;
					deltap_scale = max(1e-15, x.NormInfinity()*1e-4);
					updateWeightsMatrix(newresidue, mode);
					residue = buildNormalEquation(GptListByImages, N11, N12, N22, L1, L2, deltap_scale);

					found = ( L1.NormInfinity() + L2.NormInfinity() <= minResidue );
					//update damping factor
					damping1 *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
					damping2 *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
					damping_speed = 2.0;
					decrease = true;
				} else {
					// cancel the update
					updateSensorModels(-deltap);
					updateCoordinates(GptListByImages, -deltac);
					damping1 *= damping_speed;
					damping2 *= damping_speed;
					damping_speed *= 1.0;
				}
			}
		} while (!decrease && !found);
		++iter;
	}
}


void ossimBlockAdjustment::solve2LeastSquares(NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
				   NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2,
				   NEWMAT::ColumnVector &deltap, NEWMAT::ColumnVector &deltac)
{
	NEWMAT::Matrix A;
	NEWMAT::Matrix r;
	A = N22 - N12.t() * invert(N11) * N12;
	r = L2 - N12.t() * invert(N11)*L1;

	double maxdiag=0.0;
	for(int d=1;d<=A.Nrows();++d) {
		if (maxdiag < A(d,d)) maxdiag = A(d,d);
	}
	double damping = 1e-3 * maxdiag;
	double olddamping = 0.0;
	for(int d=1;d <= A.Nrows();++d) A(d,d) += damping - olddamping;

	deltac = invert(A)*r;
	deltap = invert(N11) * (L1 - N12 * deltac);
}
/*!
* solves Ax = r , with A symmetric positive definite
* A can be rank deficient
* size of A is typically between 10 and 100 rows
*/
NEWMAT::ColumnVector ossimBlockAdjustment::solveLeastSquares(NEWMAT::SymmetricMatrix& A,  NEWMAT::ColumnVector& r)const
{
	NEWMAT::ColumnVector x = invert(A)*r;
	return x;
}
/*!
* solves Ax = r , with A symmetric positive definite
* A can be rank deficient
* size of A is typically between 10 and 100 rows
*/
//NEWMAT::ColumnVector 
//ossimBlockAdjustment::solveLeastSquares(NEWMAT::Matrix& A,  NEWMAT::ColumnVector& r)const
//{
//	NEWMAT::ColumnVector x = invert(A)*r;
//	return x;
//}
NEWMAT::Matrix ossimBlockAdjustment::invert(const NEWMAT::Matrix& m)const
{
	ossim_uint32 idx = 0;
	NEWMAT::DiagonalMatrix d;
	NEWMAT::Matrix u;
	NEWMAT::Matrix v;
	// decompose m.t*m which is stored in Temp into the singular values and vectors.
	//
	NEWMAT::SVD(m, d, u, v, true, true);
	// invert the diagonal
	// this is just doing the reciprical fo all diagonal components and store back int
	// d.  ths compute d inverse.
	//
	for(idx=0; idx < (ossim_uint32)d.Ncols(); ++idx)
	{
		if(d[idx] > 1e-14) //TBC : use DBL_EPSILON ?
		{
			d[idx] = 1.0/d[idx];
		}
		else
		{
			d[idx] = 0.0;
			//DEBUG TBR
			cout<<"warning: singular matrix in SVD"<<endl;
		}
	}
	//compute inverse of decomposed m;
	return v*d*u.t();
}

void ossimBlockAdjustment::updateSensorModels(NEWMAT::ColumnVector deltap)
{
	int nImages = m_SensorModleList.size();
	for(int i = 0;i < nImages;++i)
	{
		//ossimKeywordlist geom;
		//m_SensorModleList[i]->saveState(geom);
		//cout<<geom<<endl<<endl;
		int np = m_SensorModleList[i]->getNumberOfAdjustableParameters();
		NEWMAT::ColumnVector cparm(np), nparm(np);
		//get current adjustment (between -1 and 1 normally) and convert to ColumnVector
		ossimAdjustmentInfo cadj;
		m_SensorModleList[i]->getAdjustment(cadj);
		std::vector< ossimAdjustableParameterInfo >& parmlist = cadj.getParameterList();
		for(int n=0;n<np;++n)
		{
			cparm(n+1) = parmlist[n].getParameter();
		}
		for(int j = 0;j < np;++j)
		{
			nparm(j + 1) = cparm(j + 1) + deltap(m_FirstParaIndices[i] + j + 1);
		}
		for(int n=0;n<np;++n)
		{
			m_SensorModleList[i]->setAdjustableParameter(n, nparm(n+1), false); //do not update now, wait
		}
		m_SensorModleList[i]->updateModel();
		//m_SensorModleList[i]->saveState(geom);
		//cout<<geom<<endl<<endl;
	}
}

void ossimBlockAdjustment::updateCoordinates(vector< vector< ossimBlockTieGpt >> &GptListByImages, NEWMAT::ColumnVector deltac)
{
	int nc = deltac.Nrows() / 3;

	for(int i = 0;i < static_cast<int>(GptListByImages.size());++i)
	{
		for(int j = 0;j < static_cast<int>(GptListByImages[i].size());++j)
		{
			if(GptListByImages[i][j].m_nUnknownIndex > -1)
			{
				ossimGpt gpt;
				gpt.lat = GptListByImages[i][j].getGroundPoint().lat + 0.001 * deltac(1 + 3 * GptListByImages[i][j].m_nUnknownIndex + 0);
				gpt.lon = GptListByImages[i][j].getGroundPoint().lon + 0.001 * deltac(1 + 3 * GptListByImages[i][j].m_nUnknownIndex + 1);
				gpt.hgt = GptListByImages[i][j].getGroundPoint().hgt + deltac(1 + 3 * GptListByImages[i][j].m_nUnknownIndex + 2);
				GptListByImages[i][j].setGroundPoint(gpt);
			}
		}
	}
}

NEWMAT::ColumnVector ossimBlockAdjustment::getResidue(const vector< vector< ossimBlockTieGpt >>& GptListByImages)
{
	int nImages = m_SensorModleList.size();
	NEWMAT::ColumnVector residue;
	int nTotalEquation = 0;
	for(int i = 0;i < nImages;++i)
	{
		nTotalEquation += 2 * GptListByImages[i].size();
	}
	residue.ReSize(nTotalEquation);
	residue = 0.0;
	unsigned long c=1;
	for(int i = 0;i < nImages;++i)
	{
		for(int j = 0;j < static_cast<int>(GptListByImages[i].size());++j)
		{
			ossimDpt resIm;
			resIm = GptListByImages[i][j].tie - m_SensorModleList[i]->forward(GptListByImages[i][j]);
			residue(c++) = resIm.x;
			residue(c++) = resIm.y;
		}
	}
	return residue;
}

void ossimBlockAdjustment::saveBlockGpt(vector<ossimBlockTieGpt> gptList, ossimString outFile, ossimMapProjection* transMerc/* = NULL*/)
{
	//输出剔除粗差点后的控制点文件
	fstream out;
	out.open(outFile.c_str(), ios_base::out);
	out.setf(ios::fixed, ios::floatfield);
	out.precision(6);
	for(int i=0;i < static_cast<int>(gptList.size());i++)
	{
		ossimDpt dpt = transMerc->forward(gptList[i]);

		out<<gptList[i].GcpNumberID
			<<"\t"<<gptList[i].getImagePoint().x
			<<"\t"<<gptList[i].getImagePoint().y
			<<"\t"<<dpt.x
			<<"\t"<<dpt.y
			<<"\t"<<gptList[i].getGroundPoint().hgt<<"\n";
	}
	out.close();
}

void ossimBlockAdjustment::updateWeightsMatrix(const NEWMAT::ColumnVector& newresidue, RobustMode mode/* = NONE*/)
{
	if(RobustMode::HUBER == mode)
		updateWeightsMatrixHuber(newresidue);
	else;
}

void ossimBlockAdjustment::updateWeightsMatrixHuber(const NEWMAT::ColumnVector& newresidue, double coeff/* = 2.0*/)
{
	int num = m_wgtMatrix.Ncols();
	double theta = newresidue.SumSquare() / num;
	double c = sqrt(theta) * coeff;
	for(int i = 0;i < num;i++)
	{
		double v = fabs(newresidue.element(i));
		if(v <= c)
			m_wgtMatrix.element(i) = v * v / 2.0;
		else
			m_wgtMatrix.element(i) = c * v - 0.5 * c *c;
	}
}
#pragma warning(pop)