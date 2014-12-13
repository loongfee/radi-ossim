#include <cstdlib>
#include <radi/radiBlockAdjustment.h>
#include <newmat/newmatap.h>
#pragma comment(lib, "newmat.lib")

namespace ossimplugins
{

radiBlockAdjustment::radiBlockAdjustment()
{
   
}

radiBlockAdjustment::~radiBlockAdjustment()
{
	
}

void radiBlockAdjustment::addSensorModel(int id, ossimSensorModel* sensorModel)
{
	int npara = sensorModel->getNumberOfAdjustableParameters();
	if(m_ParamPosMap.size() == 0)
	{
		m_ParamPosMap[id].begin = 0;
		m_ParamPosMap[id].end = m_ParamPosMap[id].begin + npara;
	}
	else
	{
		m_ParamPosMap[id].begin = m_ParamPosMap.rbegin()->second.end;
		m_ParamPosMap[id].end = m_ParamPosMap[id].begin + npara;
	}
	m_SensorModleMap[id] = sensorModel;
	//m_SensorModleMap.insert(pair<int, ossimSensorModel*>(id, sensorModel));
}

// give the unknown corresponding image points initial values
int radiBlockAdjustment::initiateGpt(vector< radiBlockTieGpt > &GptList)
{
	//int nPoints = GptList.size();
	//ossimGpt gpt;

	//m_nUnknownPoints = 0;
	//for(int i = 0;i < nPoints;++i)
	//{
	//	if(0 == GptList[i].m_nType)
	//	{// if the point is an unknown corresponding image point
	//		gpt = m_SensorModleList[GptList[i].m_ImageIndices[0]]->inverse(GptList[i].m_DptList[0]);
	//		GptList[i].setImagePoint(GptList[i].m_DptList[0]);
	//		GptList[i].setGroundPoint(gpt);
	//		GptList[i].m_nUnknownIndex = m_nUnknownPoints++;
	//	}
	//	else
	//	{
	//		ossimDpt imagepoint,cimagepoint;
	//		ossimGpt goundpoint,tGpt;
	//		imagepoint = GptList[i].getImagePoint();
	//		goundpoint = GptList[i].getGroundPoint();
	//		tGpt = m_SensorModleList[GptList[i].m_ImageIndices[0]]->m_proj->inverse(ossimDpt(goundpoint.lat,goundpoint.lon));
	//		tGpt.hgt = GptList[i].hgt;
	//		GptList[i].setImagePoint(GptList[i].m_DptList[0]);
	//		GptList[i].setGroundPoint(tGpt);
	//		GptList[i].m_nUnknownIndex = -1;
	//	}
	//}
	return m_nUnknownPoints;
}

// give the unknown corresponding image points initial values
int radiBlockAdjustment::initiateGpt(radiBlockTieGptSet &GptList)
{
	int nPoints = (int)GptList.refTiePoints().size();
	ossimGpt gpt;

	m_nUnknownPoints = 0;
	for(int i = 0;i < nPoints;++i)
	{
		int slaveId = GptList.refTiePoints()[i]->getSlaveId();
		if(radiBlockTieGpt::point_type::unknown_tie_image_points == GptList.getTiePoints()[i]->m_nType)
		{
			// if the point is an unknown corresponding image point
			m_SensorModleMap[GptList.refTiePoints()[i]->m_DptList[0].first]->lineSampleToWorld(GptList.refTiePoints()[i]->m_DptList[0].second, gpt);
			//m_SensorModleMap[masterId]->lineSampleToWorld(GptList.refTiePoints()[i]->getImagePoint(), gpt);
			if (ossimElevManager::instance())
			{
				gpt.hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(gpt);
			}
			GptList.refTiePoints()[i]->setGroundPoint(gpt);
			GptList.refTiePoints()[i]->m_nUnknownIndex = m_nUnknownPoints++;
		}
		else if(radiBlockTieGpt::point_type::known_ground_control_points == GptList.getTiePoints()[i]->m_nType)
		{
			if (!(NULL == m_SensorModleMap[slaveId]->m_proj) && !m_SensorModleMap[slaveId]->m_proj->isGeographic())
			{
				ossimDpt imagepoint,cimagepoint;
				ossimGpt goundpoint,tGpt;
				imagepoint = GptList.refTiePoints()[i]->getImagePoint();
				goundpoint = GptList.refTiePoints()[i]->getGroundPoint();
				tGpt = m_SensorModleMap[slaveId]->m_proj->inverse(ossimDpt(goundpoint.lat,goundpoint.lon));
				tGpt.hgt = GptList.refTiePoints()[i]->hgt;
				if (ossimElevManager::instance())
				{
					gpt.hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(gpt);
				}
				GptList.refTiePoints()[i]->setGroundPoint(tGpt);
			}
			else
			{
				ossimGpt goundpoint = GptList.refTiePoints()[i]->getGroundPoint();
				if (ossimElevManager::instance())
				{
					goundpoint.hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(goundpoint);
					GptList.refTiePoints()[i]->setGroundPoint(goundpoint);
				}
			}
			GptList.refTiePoints()[i]->m_nUnknownIndex = -1;
		}
	}
	return m_nUnknownPoints;
}

// find the gcps for each image and put them into a vector container
vector< vector< radiBlockTieGpt >> radiBlockAdjustment::classifyGptByImages(vector< radiBlockTieGpt > GptList)
{
	vector< vector< radiBlockTieGpt > > GptListByImages;
	//int nImages = m_SensorModleList.size();
	//for(int i = 0;i < nImages;++i)
	//{
	//	vector< radiBlockTieGpt > gptListTmp;
	//	for(int j = 0;j < static_cast<int>(GptList.size());++j)
	//	{
	//		for(int k = 0;k < static_cast<int>(GptList[j].m_ImageIndices.size());++k)
	//		{
	//			if(GptList[j].m_ImageIndices[k] == i)
	//			{
	//				radiBlockTieGpt blockTieGpt = GptList[j];
	//				blockTieGpt.setImagePoint(blockTieGpt.m_DptList[k]);
	//				gptListTmp.push_back(blockTieGpt);
	//			}
	//		}
	//	}
	//	GptListByImages.push_back(gptListTmp);
	//}
	return GptListByImages;
}
std::map<int, radiBlockTieGptSet > radiBlockAdjustment::classifyGptByImages(radiBlockTieGptSet GptList)
{
	int nImages = (int)m_SensorModleMap.size();
	std::map<int, radiBlockTieGptSet > GptMapByImages;
	std::map<int, ossimSensorModel*>::iterator iter;
	for (iter = m_SensorModleMap.begin();iter != m_SensorModleMap.end();++iter)
	{
		int imageId = iter->first;
		//radiBlockTieGptSet gptListTmp;
		for(int j = 0;j < static_cast<int>(GptList.refTiePoints().size());++j)
		{
			if (radiBlockTieGpt::point_type::unknown_tie_image_points == GptList.refTiePoints()[j]->m_nType)
			{
				for (std::vector<std::pair<int, ossimDpt> >::iterator iterDpt = GptList.refTiePoints()[j]->m_DptList.begin();
					iterDpt != GptList.refTiePoints()[j]->m_DptList.end();++iterDpt)
				{
					int imageId1 = iterDpt->first;
					if (imageId1 == imageId)
					{
						//gptListTmp.addTiePoint(GptList.refTiePoints()[j]);
						ossimRefPtr<radiBlockTieGpt> blockTiePoint = new radiBlockTieGpt;
						*blockTiePoint = *GptList.refTiePoints()[j];
						blockTiePoint->setImagePoint(iterDpt->second);
						blockTiePoint->setSlaveId(imageId1);
						GptMapByImages[imageId].addTiePoint(blockTiePoint);
					}
				}
			}
			else if (radiBlockTieGpt::point_type::known_ground_control_points == GptList.refTiePoints()[j]->m_nType)
			{
				if (GptList.refTiePoints()[j]->getSlaveId() == imageId)
				{
					ossimRefPtr<radiBlockTieGpt> blockTiePoint = new radiBlockTieGpt;
					*blockTiePoint = *GptList.refTiePoints()[j];
					GptMapByImages[imageId].addTiePoint(blockTiePoint);
				}
			}
		}
		//GptListByImages[imageId] = gptListTmp;
	}
	return GptMapByImages;
}

// build the block adjustment error equation
void radiBlockAdjustment::buildErrorEquation(const vector< vector< radiBlockTieGpt >>& GptListByImages,
											 NEWMAT::Matrix &AA, NEWMAT::Matrix &BB, NEWMAT::ColumnVector &LL,
											 double pstep_scale)
{
	//int nImages = m_SensorModleList.size();
	//int nTotalParam = m_FirstParaIndices.back();
	//int nTotalEquation = 0;
	//int nUnknownCoordinate = m_nUnknownPoints * 3;
	//for(int i = 0;i < nImages;++i)
	//{
	//	nTotalEquation += 2 * GptListByImages[i].size();
	//}

	//AA.ReSize(nTotalEquation, nTotalParam);
	//BB.ReSize(nTotalEquation, nUnknownCoordinate);
	//LL.ReSize(nTotalEquation);
	//AA = 0.0;
	//BB = 0.0;
	//LL = 0.0;


	//int nEquationIndex = 0;
	//for(int i = 0;i < nImages;++i)
	//{
	//	for(int j = 0;j < static_cast<int>(GptListByImages[i].size());++j)
	//	{
	//		NEWMAT::Matrix A;
	//		NEWMAT::Matrix B;
	//		NEWMAT::ColumnVector L;
	//		// use one gcp to build two error equations
	//		m_SensorModleList[i]->buildErrorEquation(GptListByImages[i][j], GptListByImages[i][j].m_nType, A, B, L, pstep_scale);
	//		
	//		// add the two equations to the block adjustment error equations set
	//		for(int k = 0;k < static_cast<int>(m_SensorModleList[i]->getNumberOfAdjustableParameters());++k)
	//		{
	//			AA.element(0 + nEquationIndex, m_FirstParaIndices[i] + k) = A.element(0, k);
	//			AA.element(1 + nEquationIndex, m_FirstParaIndices[i] + k) = A.element(1, k);
	//		}
	//		if(GptListByImages[i][j].m_nUnknownIndex > -1)
	//		{
	//			for(int k = 0;k < 3;++k)
	//			{
	//				BB.element(0 + nEquationIndex, GptListByImages[i][j].m_nUnknownIndex * 3 + k) = B.element(0, k);
	//				BB.element(1 + nEquationIndex, GptListByImages[i][j].m_nUnknownIndex * 3 + k) = B.element(1, k);
	//			}

	//		}

	//		LL.element(0 + nEquationIndex) = L.element(0);
	//		LL.element(1 + nEquationIndex) = L.element(1);

	//		nEquationIndex += 2;
	//	}
	//}
}
void radiBlockAdjustment::buildErrorEquation(const std::map<int, radiBlockTieGptSet>& GptMapByImages,
	NEWMAT::Matrix &AA, NEWMAT::Matrix &BB, NEWMAT::ColumnVector &LL,
	double pstep_scale)
{
	int nImages = m_SensorModleMap.size();
	int nTotalParam = m_ParamPosMap.rbegin()->second.end;
	int nUnknownCoordinate = m_nUnknownPoints * 2;

	int nTotalEquation = 0;
	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = GptMapByImages.begin();
		iter != GptMapByImages.end();++iter)
	{
		nTotalEquation += 2 * (int)iter->second.getTiePoints().size();
	}

	AA.ReSize(nTotalEquation, nTotalParam);
	BB.ReSize(nTotalEquation, nUnknownCoordinate);
	LL.ReSize(nTotalEquation);
	AA = 0.0;
	BB = 0.0;
	LL = 0.0;


	int nEquationIndex = 0;
	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = GptMapByImages.begin();
		iter != GptMapByImages.end();++iter)
	{
		int imageId = iter->first;
		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
		{
			NEWMAT::Matrix A;
			NEWMAT::Matrix B;
			NEWMAT::ColumnVector L;
			// use one gcp to build two error equations
			m_SensorModleMap[imageId]->buildErrorEquation(*iter->second.getTiePoints()[j], iter->second.getTiePoints()[j]->m_nType, A, B, L, pstep_scale);

			// add the two equations to the block adjustment error equations set
			for(int k = 0;k < static_cast<int>(m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
			{
				AA.element(0 + nEquationIndex, m_ParamPosMap[imageId].begin + k) = A.element(0, k);
				AA.element(1 + nEquationIndex, m_ParamPosMap[imageId].begin + k) = A.element(1, k);
			}
			if(iter->second.getTiePoints()[j]->m_nUnknownIndex > -1)
			{
				for(int k = 0;k < 2;++k)
				{
					BB.element(0 + nEquationIndex, iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + k) = B.element(0, k);
					BB.element(1 + nEquationIndex, iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + k) = B.element(1, k);
				}

			}

			LL.element(0 + nEquationIndex) = L.element(0);
			LL.element(1 + nEquationIndex) = L.element(1);

			nEquationIndex += 2;
		}
	}
}

NEWMAT::ColumnVector radiBlockAdjustment::buildNormalEquation(const vector< vector< radiBlockTieGpt >>& GptListByImages, 
											  NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
											  NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2, double pstep_scale)
{
	NEWMAT::Matrix A;
	NEWMAT::Matrix B;
	NEWMAT::ColumnVector L;
	//buildErrorEquation(GptListByImages, A, B, L, pstep_scale);

	////ofstream fs("D:\\loong\\Programs\\BlockAdjustment\\Release\\matrix1.txt");
	////fs<<"A:"<<endl<<A<<endl<<"B:"<<endl<<B<<endl<<"L:"<<endl<<L<<endl;
	////fs.close();

	//int nobs = A.Nrows();
	//int np1 = A.Ncols();
	//int np2 = B.Ncols();

	//N11.ReSize(np1, np1);
	//N12.ReSize(np1, np2);
	//N22.ReSize(np2, np2);
	//L1.ReSize(np1);
	//L2.ReSize(np2);

	//N11 = A.t() * m_wgtMatrix * A;
	//N12 = A.t() * m_wgtMatrix * B;
	//N22 = B.t() * m_wgtMatrix * B;
	//L1 = A.t() * m_wgtMatrix * L;
	//L2 = B.t() * m_wgtMatrix * L;

	return L;
}

NEWMAT::ColumnVector radiBlockAdjustment::buildNormalEquation(const std::map<int, radiBlockTieGptSet>& GptMapByImages, 
	NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
	NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2, double pstep_scale)
{
	NEWMAT::Matrix A;
	NEWMAT::Matrix B;
	NEWMAT::ColumnVector L;
	buildErrorEquation(GptMapByImages, A, B, L, pstep_scale);

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

void radiBlockAdjustment::adjustment(vector< radiBlockTieGpt > GptList, RobustMode mode/* = NONE*/)
{
	//int nImages = m_SensorModleMap.size();
	//int nTotalParam = m_FirstParaIndices.back();
	//int nUnknownPoints = initiateGpt(GptList);
	//int nUnknownCoordinate = m_nUnknownPoints * 3;
	//
	//vector< vector< radiBlockTieGpt >> GptListByImages = classifyGptByImages( GptList );

	//int nTotalEquation = 0;
	//for(int i = 0;i < nImages;++i)
	//{
	//	nTotalEquation += 2 * GptListByImages[i].size();
	//}

	////saveBlockGpt(GptListByImages[0], "D:\\loong\\Programs\\BlockAdjustment\\Release\\gpt1.txt", m_SensorModleList[0]->m_proj);

	////setup initail values
	//int iter=0;
	//int iter_max = 200;    //ww1130
	//double minResidue = 1e-6; //TBC
	//double minDelta = 1e-6; //TBC
	////build Least Squares initial normal equation
	//// don't waste memory, add samples one at a time
	//NEWMAT::Matrix N11;
	//NEWMAT::Matrix N12; 
	//NEWMAT::Matrix N22;
	//NEWMAT::ColumnVector L1;
	//NEWMAT::ColumnVector L2;
	//NEWMAT::ColumnVector residue;
	//NEWMAT::SymmetricMatrix A;
	//NEWMAT::ColumnVector L;


	//m_wgtMatrix.ReSize(nTotalEquation);
	//m_wgtMatrix = 1;

	//double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1

	//residue = buildNormalEquation(GptListByImages, N11, N12, N22, L1, L2,deltap_scale);

	//residue = getResidue(GptListByImages);
	//double ki2=residue.SumSquare();

	//int nobs = residue.Nrows() / 2;

	//double damping_speed = 1.0;
	//double maxdiag1=0.0;
	//double maxdiag2=0.0;
	//double damping1;
	//double damping2;

	////find max diag element for N11
	//maxdiag1=0.0;
	//maxdiag2=0.0;
	//for(int d=1;d<=N11.Nrows();++d) {if (maxdiag1 < N11(d,d)) maxdiag1 = N11(d,d);}
	//for(int d=1;d<=N22.Nrows();++d) {if (maxdiag2 < N22(d,d)) maxdiag2 = N22(d,d);}
	//damping1 = damping_speed * 1e-3 * maxdiag1;
	//damping2 = damping_speed * 1e-3 * maxdiag2;

	//double olddamping1 = 0.0;
	//double olddamping2 = 0.0;

	//bool found = false;

	//while ( (!found) && (iter < iter_max) ) //non linear optimization loop
	//{
	//	bool decrease = false;
	//	do
	//	{
	//		int n1 = N11.Nrows();
	//		int n2 = N22.Nrows();
	//		A.ReSize(n1 + n2);
	//		L.ReSize(n1 + n2);
	//		for(int i = 0;i < n1 + n2;++i)
	//		{
	//			for(int j = 0;j < n1 + n2;++j)
	//			{
	//				if(i < n1)
	//				{
	//					if(j < n1)
	//					{
	//						A.element(i,j) = N11.element(i,j);
	//					}
	//					else
	//					{
	//						A.element(i,j) = N12.element(i, j - n1);
	//					}
	//					L.element(i) = L1.element(i);
	//				}
	//				else
	//				{
	//					if(j < n1)
	//					{
	//						A.element(i,j) = N12.element(j, i - n1);
	//					}
	//					else
	//					{
	//						A.element(i,j) = N22.element(i - n1, j - n1);
	//					}
	//					L.element(i) = L2.element(i - n1);
	//				}
	//			}
	//		}

	//		//fstream fs;
	//		//fs.open("D:\\loong\\Programs\\BlockAdjustment\\Release\\matrix.txt",ios_base::out);
	//		//fs<<"N11:"<<endl<<N11<<endl<<"N12:"<<endl<<N12<<endl<<"N22:"<<endl<<N22<<endl<<"L1:"<<endl<<L1<<endl<<"L2:"<<endl<<L2<<endl;
	//		//fs.close();
	//		//fs.open("D:\\loong\\Programs\\BlockAdjustment\\Release\\matrix2.txt",ios_base::out);
	//		//fs<<"A:"<<endl<<A<<endl<<"L:"<<endl<<L<<endl;
	//		//fs.close();


	//		//add damping update to normal matrix
	//		for(int d=1;d<=N11.Nrows();++d) A(d,d) += damping1 - olddamping1;
	//		for(int d=1;d<=N22.Nrows();++d) A(d+N11.Nrows(),d+N11.Nrows()) += damping2 - olddamping2;
	//		olddamping1 = damping1;
	//		olddamping2 = damping2;

	//		NEWMAT::ColumnVector deltap;
	//		NEWMAT::ColumnVector deltac; //= solveLeastSquares(A, projResidue);
	//		deltap.ReSize(N11.Nrows());
	//		deltac.ReSize(N22.Nrows());
	//		//solve2LeastSquares(N11, N12, N22, L1, L2, deltap, deltac);


	//		/*fstream fs;
	//		fs.open("H:\\AlosData\\matrix_block.txt", ios_base::out);
	//		for(int i = 0;i < A.Nrows();i++)
	//		{
	//			for(int j = 0;j < A.Ncols();j++)
	//			{
	//				fs<<A.element(i,j)<<"\t";
	//			}
	//			fs<<endl;
	//		}
	//		fs<<endl<<endl;
	//		for(int i = 0;i < L.Nrows();i++)
	//		{
	//			fs<<L.element(i)<<endl;
	//		}
	//		fs.close();*/


	//		NEWMAT::ColumnVector x = solveLeastSquares(A, L);
	//		for(int i = 0;i < N11.Nrows();++i)
	//			deltap.element(i) = x.element(i);
	//		for(int i = 0;i < N22.Nrows();++i)
	//			deltac.element(i) = x.element(N11.Nrows() + i);

	//		cout<<x<<endl<<endl;
	//		if (x.NormFrobenius() <= x.Nrows() * minDelta)
	//		{
	//			found = true;
	//		} else {
	//			//update adjustment
	//			updateSensorModels(deltap);
	//			updateCoordinates(GptListByImages, deltac);

	//			NEWMAT::ColumnVector newresidue = getResidue(GptListByImages);
	//			double newki2 = newresidue.SumSquare();
	//			double res_reduction = (ki2 - newki2) /
	//				((deltap.t()*(deltap*damping1 + L1)).AsScalar()
	//				+ (deltac.t()*(deltac*damping2 + L2)).AsScalar());

	//			//DEBUG TBR
	//			cout<<sqrt(newki2/nobs)<<" the "<<iter<<"th iteration"<<endl;
	//			cout.flush();
	//			if (res_reduction > 0)
	//			{
	//				//accept new parms
	//				ki2=newki2;
	//				deltap_scale = max(1e-15, x.NormInfinity()*1e-4);
	//				updateWeightsMatrix(newresidue, mode);
	//				residue = buildNormalEquation(GptListByImages, N11, N12, N22, L1, L2, deltap_scale);

	//				found = ( L1.NormInfinity() + L2.NormInfinity() <= minResidue );
	//				//update damping factor
	//				damping1 *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
	//				damping2 *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
	//				damping_speed = 2.0;
	//				decrease = true;
	//			} else {
	//				// cancel the update
	//				updateSensorModels(-deltap);
	//				updateCoordinates(GptListByImages, -deltac);
	//				damping1 *= damping_speed;
	//				damping2 *= damping_speed;
	//				damping_speed *= 1.0;
	//			}
	//		}
	//	} while (!decrease && !found);
	//	++iter;
	//}
}

void radiBlockAdjustment::adjustment(radiBlockTieGptSet& GptList, RobustMode mode/* = NONE*/)
{
	return sparselm_adjustment( GptList, mode);
	////return dogleg_adjustment( GptList, mode);
	//int nImages = m_SensorModleMap.size();
	//int nTotalParam = m_ParamPosMap.rbegin()->second.end;
	//int nUnknownPoints = initiateGpt(GptList);
	//int nUnknownCoordinate = m_nUnknownPoints * 2;
	//
	//std::map<int, radiBlockTieGptSet> GptMapByImages = classifyGptByImages( GptList );

	//int nTotalEquation = 0;
	//for(std::map<int, radiBlockTieGptSet>::iterator iter = GptMapByImages.begin();
	//	iter != GptMapByImages.end();++iter)
	//{
	//	nTotalEquation += 2 * (int)iter->second.getTiePoints().size();
	//}

	////saveBlockGpt(GptListByImages[0], "D:\\loong\\Programs\\BlockAdjustment\\Release\\gpt1.txt", m_SensorModleList[0]->m_proj);

	////setup initail values
	//int iter=0;
	//int iter_max = 200;    //ww1130
	//double minResidue = 1e-6; //TBC
	//double minDelta = 1e-6; //TBC
	////build Least Squares initial normal equation
	//// don't waste memory, add samples one at a time
	//NEWMAT::Matrix N11;
	//NEWMAT::Matrix N12; 
	//NEWMAT::Matrix N22;
	//NEWMAT::ColumnVector L1;
	//NEWMAT::ColumnVector L2;
	//NEWMAT::ColumnVector residue;
	//NEWMAT::SymmetricMatrix A;
	//NEWMAT::ColumnVector L;


	//m_wgtMatrix.ReSize(nTotalEquation);
	//m_wgtMatrix = 1;

	//double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1

	//residue = buildNormalEquation(GptMapByImages, N11, N12, N22, L1, L2,deltap_scale);

	//residue = getResidue(GptMapByImages);
	//double ki2=residue.SumSquare();

	//int nobs = residue.Nrows() / 2;
	//ki2 /= nobs;

	//double damping_speed = 1.0;
	//double maxdiag1=0.0;
	//double maxdiag2=0.0;
	//double damping1;
	//double damping2;

	////find max diag element for N11
	//maxdiag1=0.0;
	//maxdiag2=0.0;
	//for(int d=1;d<=N11.Nrows();++d) {if (maxdiag1 < N11(d,d)) maxdiag1 = N11(d,d);}
	//for(int d=1;d<=N22.Nrows();++d) {if (maxdiag2 < N22(d,d)) maxdiag2 = N22(d,d);}
	//damping1 = damping_speed * 1e-3 * maxdiag1;
	//damping2 = damping_speed * 1e-3 * maxdiag2;

	//double olddamping1 = 0.0;
	//double olddamping2 = 0.0;

	//bool found = false;

	//while ( (!found) && (iter < iter_max) ) //non linear optimization loop
	//{
	//	bool decrease = false;
	//	do
	//	{
	//		int n1 = N11.Nrows();
	//		int n2 = N22.Nrows();
	//		A.ReSize(n1 + n2);
	//		L.ReSize(n1 + n2);
	//		for(int i = 0;i < n1 + n2;++i)
	//		{
	//			for(int j = 0;j < n1 + n2;++j)
	//			{
	//				if(i < n1)
	//				{
	//					if(j < n1)
	//					{
	//						A.element(i,j) = N11.element(i,j);
	//					}
	//					else
	//					{
	//						A.element(i,j) = N12.element(i, j - n1);
	//					}
	//					L.element(i) = L1.element(i);
	//				}
	//				else
	//				{
	//					if(j < n1)
	//					{
	//						A.element(i,j) = N12.element(j, i - n1);
	//					}
	//					else
	//					{
	//						A.element(i,j) = N22.element(i - n1, j - n1);
	//					}
	//					L.element(i) = L2.element(i - n1);
	//				}
	//			}
	//		}

	//		//fstream fs;
	//		//fs.open("D:\\loong\\Programs\\BlockAdjustment\\Release\\matrix.txt",ios_base::out);
	//		//fs<<"N11:"<<endl<<N11<<endl<<"N12:"<<endl<<N12<<endl<<"N22:"<<endl<<N22<<endl<<"L1:"<<endl<<L1<<endl<<"L2:"<<endl<<L2<<endl;
	//		//fs.close();
	//		//fs.open("D:\\loong\\Programs\\BlockAdjustment\\Release\\matrix2.txt",ios_base::out);
	//		//fs<<"A:"<<endl<<A<<endl<<"L:"<<endl<<L<<endl;
	//		//fs.close();


	//		//add damping update to normal matrix
	//		for(int d=1;d<=N11.Nrows();++d) A(d,d) += damping1 - olddamping1;
	//		for(int d=1;d<=N22.Nrows();++d) A(d+N11.Nrows(),d+N11.Nrows()) += damping2 - olddamping2;
	//		olddamping1 = damping1;
	//		olddamping2 = damping2;

	//		NEWMAT::ColumnVector deltap;
	//		NEWMAT::ColumnVector deltac; //= solveLeastSquares(A, projResidue);
	//		deltap.ReSize(N11.Nrows());
	//		deltac.ReSize(N22.Nrows());
	//		//solve2LeastSquares(N11, N12, N22, L1, L2, deltap, deltac);


	//		/*fstream fs;
	//		fs.open("H:\\AlosData\\matrix_block.txt", ios_base::out);
	//		for(int i = 0;i < A.Nrows();i++)
	//		{
	//			for(int j = 0;j < A.Ncols();j++)
	//			{
	//				fs<<A.element(i,j)<<"\t";
	//			}
	//			fs<<endl;
	//		}
	//		fs<<endl<<endl;
	//		for(int i = 0;i < L.Nrows();i++)
	//		{
	//			fs<<L.element(i)<<endl;
	//		}
	//		fs.close();*/


	//		NEWMAT::ColumnVector x = solveLeastSquares(A, L);
	//		for(int i = 0;i < N11.Nrows();++i)
	//			deltap.element(i) = x.element(i);
	//		for(int i = 0;i < N22.Nrows();++i)
	//			deltac.element(i) = x.element(N11.Nrows() + i);

	//		//cout<<x<<endl<<endl;
	//		if (x.NormFrobenius() <= x.Nrows() * minDelta)
	//		{
	//			found = true;
	//		} else {
	//			//update adjustment
	//			updateSensorModels(deltap);
	//			updateCoordinates(GptMapByImages, deltac);

	//			NEWMAT::ColumnVector newresidue = getResidue(GptMapByImages);
	//			NEWMAT::ColumnVector weightedNewResidue = m_wgtMatrix * newresidue;
	//			double newki2 = weightedNewResidue.SumSquare();
	//			double weightedCounter = 0;
	//			for (int iw = 0;iw < m_wgtMatrix.Ncols();++iw)
	//			{
	//				weightedCounter += fabs(m_wgtMatrix.element(iw));
	//			}
	//			weightedCounter /= 2.0;
	//			newki2 /= weightedCounter;
	//			double res_reduction = (ki2 - newki2) /
	//				((deltap.t()*(deltap*damping1 + L1)).AsScalar()
	//				+ (deltac.t()*(deltac*damping2 + L2)).AsScalar());
	//			//DEBUG TBR
	//			//cout<<sqrt(newki2/nobs)<<" the "<<iter<<"th iteration"<<endl;
	//			cout<<weightedCounter<<" "<<nobs<<" ";
	//			cout<<sqrt(newki2)<<" the "<<iter<<"th iteration"<<endl;
	//			cout.flush();
	//			if (res_reduction > 0)
	//			{
	//				//accept new parms
	//				ki2=newki2;
	//				deltap_scale = max(1e-15, x.NormInfinity()*1e-4);
	//				updateWeightsMatrix(newresidue, mode);
	//				residue = buildNormalEquation(GptMapByImages, N11, N12, N22, L1, L2, deltap_scale);

	//				found = ( L1.NormInfinity() + L2.NormInfinity() <= minResidue );
	//				//update damping factor
	//				damping1 *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
	//				damping2 *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
	//				damping_speed = 2.0;
	//				decrease = true;
	//			} else {
	//				// cancel the update
	//				updateSensorModels(-deltap);
	//				updateCoordinates(GptMapByImages, -deltac);
	//				damping1 *= damping_speed;
	//				damping2 *= damping_speed;
	//				damping_speed *= 1.0;
	//			}
	//		}
	//	} while (!decrease && !found);
	//	++iter;
	//}
}

//void radiBlockAdjustment::optimizerCallback(const double*   p,
//							  double*         x,
//							  cholmod_sparse* Jt,
//							  void* cookie )
//{
//	optimzeDataStruct* pData = (optimzeDataStruct*)cookie;
//	// These are convenient so that I only apply the casts once
//	int*    Jrowptr = (int*)Jt->p;
//	int*    Jcolidx = (int*)Jt->i;
//	double* Jval    = (double*)Jt->x;
//
//	int iJacobian = 0;
//#define STORE_JACOBIAN(col, g)                  \
//	do                                      \
//	{                                       \
//	Jcolidx[ iJacobian ] = col;           \
//	Jval   [ iJacobian ] = g;             \
//	iJacobian++;                          \
//	} while(0)
//
//
//	double norm2_x = 0.0;
//
//	int nImages = pData->pBa->m_SensorModleMap.size();
//	int nTotalParam = pData->pBa->m_ParamPosMap.rbegin()->second.end;
//	int nUnknownCoordinate = pData->pBa->m_nUnknownPoints * 2;
//
//	int nTotalEquation = 0;
//	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = pData->GptMapByImages->begin();
//		iter != pData->GptMapByImages->end();++iter)
//	{
//		nTotalEquation += 2 * (int)iter->second.getTiePoints().size();
//	}
//	// update parameters and coordinates
//	for(std::map<int, radiBlockTieGptSet>::iterator iter = pData->GptMapByImages->begin();
//		iter != pData->GptMapByImages->end();++iter)
//	{
//		int imageId = iter->first;
//		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
//		{
//			for(int k = 0;k < static_cast<int>(pData->pBa->m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
//			{
//				pData->pBa->m_SensorModleMap[imageId]->setAdjustableParameter(k, p[pData->pBa->m_ParamPosMap[imageId].begin + k]);
//				if(iter->second.getTiePoints()[j]->m_nUnknownIndex > -1)
//				{
//					ossimGpt gptUpdate(p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 0],
//						p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 1], 0.0);
//					gptUpdate.hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(gptUpdate);
//					iter->second.refTiePoints()[j]->setGroundPoint(gptUpdate);
//				}
//			}
//		}
//	}
//
//	double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1
//	int nEquationIndex = 0;
//	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = pData->GptMapByImages->begin();
//		iter != pData->GptMapByImages->end();++iter)
//	{
//		int imageId = iter->first;
//		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
//		{
//			NEWMAT::Matrix A;
//			NEWMAT::Matrix B;
//			NEWMAT::ColumnVector L;
//			// use one gcp to build two error equations
//			pData->pBa->m_SensorModleMap[imageId]->buildErrorEquation(*iter->second.getTiePoints()[j], iter->second.getTiePoints()[j]->m_nType, A, B, L, deltap_scale);
//			x[nEquationIndex] = -L.element(0);
//			x[nEquationIndex+1] = -L.element(1);
//			norm2_x += x[nEquationIndex]*x[nEquationIndex] + x[nEquationIndex+1]*x[nEquationIndex+1];
//			// add the two equations to the block adjustment error equations set
//
//			// In this sample problem, every measurement depends on every element of the
//			// state vector, so I loop through all the state vectors here. In practice
//			// libdogleg is meant to be applied to sparse problems, where this internal
//			// loop would be MUCH shorter than Nstate long
//			Jrowptr[nEquationIndex] = iJacobian;
//			for(int k = 0;k < static_cast<int>(pData->pBa->m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
//			{
//				STORE_JACOBIAN( pData->pBa->m_ParamPosMap[imageId].begin + k, A.element(0, k) );
//			}
//			if(iter->second.getTiePoints()[j]->m_nUnknownIndex > -1)
//			{
//				for(int kk = 0;kk < 2;++kk)
//				{
//					STORE_JACOBIAN( nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + kk, B.element(0, kk) );
//				}
//			}
//			Jrowptr[nEquationIndex+1] = iJacobian;
//			for(int k = 0;k < static_cast<int>(pData->pBa->m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
//			{
//				STORE_JACOBIAN( pData->pBa->m_ParamPosMap[imageId].begin + k, A.element(1, k) );
//			}
//			if(iter->second.getTiePoints()[j]->m_nUnknownIndex > -1)
//			{
//				for(int kk = 0;kk < 2;++kk)
//				{
//					STORE_JACOBIAN( nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + kk, B.element(1, kk) );
//				}
//			}
//
//			nEquationIndex += 2;
//		}
//	}
//	Jrowptr[nTotalEquation] = iJacobian;
//
//
//	//fprintf(stderr, "Callback finished. 2-norm is %f\n", norm2_x);
//	fprintf(stderr, "Callback finished. rms error is %lf\n", sqrt(norm2_x/(nTotalEquation/2)));
//}
//
//void radiBlockAdjustment::dogleg_adjustment(radiBlockTieGptSet& GptList, RobustMode mode/* = NONE*/)
//{
//	int nImages = m_SensorModleMap.size();
//	int nTotalParam = m_ParamPosMap.rbegin()->second.end;
//	int nUnknownPoints = initiateGpt(GptList);
//	int nUnknownCoordinate = m_nUnknownPoints * 2;
//
//	std::map<int, radiBlockTieGptSet> GptMapByImages = classifyGptByImages( GptList );
//
//	int nTotalEquation = 0;
//	for(std::map<int, radiBlockTieGptSet>::iterator iter = GptMapByImages.begin();
//		iter != GptMapByImages.end();++iter)
//	{
//		nTotalEquation += 2 * (int)iter->second.getTiePoints().size();
//	}
//
//	int Nstate = nTotalParam + nUnknownCoordinate;
//	int Nmeasurements = nTotalEquation;
//	dogleg_setDebug(0); // request debugging output from the solver
//	//dogleg_setDebug(1); // request debugging output from the solver
//	double *p = new double[Nstate];
//	for(std::map<int, radiBlockTieGptSet>::iterator iter = GptMapByImages.begin();
//		iter != GptMapByImages.end();++iter)
//	{
//		int imageId = iter->first;
//		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
//		{
//			for(int k = 0;k < static_cast<int>(m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
//			{
//				p[m_ParamPosMap[imageId].begin + k] = m_SensorModleMap[imageId]->getAdjustableParameter(k);
//				if(iter->second.getTiePoints()[j]->m_nUnknownIndex > -1)
//				{
//					ossimGpt gpt = iter->second.refTiePoints()[j]->getGroundPoint();
//					p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 0] = gpt.lat;
//					p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 1] = gpt.lon;
//				}
//			}
//		}
//	}
//	optimzeDataStruct dataStruct;
//	dataStruct.pBa = this;
//	dataStruct.GptMapByImages = &GptMapByImages;
//
//	//fprintf(stderr, "starting state:\n");
//	//for(int i=0; i<Nstate; i++)
//	//	fprintf(stderr, "  p[%d] = %f\n", i, p[i]);
//
//	// This demo problem is dense, so every measurement depends on every state
//	// variable. Thus ever element of the jacobian is non-zero
//	int Jnnz = Nmeasurements * Nstate;
//
//
//	// first, let's test our gradients. This is just a verification step to make
//	// sure the optimizerCallback() is written correctly. Normally, you would do
//	// this as a check when developing your program, but would turn this off in
//	// the final application. This will generate LOTS of output. You need to make
//	// sure that the reported and observed gradients match (the relative error is
//	// low)
//	fprintf(stderr, "have %d variables\n", Nstate);
//	//for(int i=0; i<Nstate; i++)
//	//{
//	//	fprintf(stderr, "checking gradients for variable %d\n", i);
//	//	dogleg_testGradient(i, p, Nstate, Nmeasurements, Jnnz, &optimizerCallback, &dataStruct);
//	//}
//
//
//	fprintf(stderr, "SOLVING:\n");
//
//	double optimum = dogleg_optimize(p, Nstate, Nmeasurements, Jnnz, &optimizerCallback, &dataStruct, NULL);
//
//	fprintf(stderr, "Done. Optimum = %f\n", sqrt(optimum/(Nmeasurements/2)));
//
//	//fprintf(stderr, "optimal state:\n");
//	//for(int i=0; i<Nstate; i++)
//	//	fprintf(stderr, "  p[%d] = %f\n", i, p[i]);
//
//	delete []p;
//}


void radiBlockAdjustment::splm_func(double *p, double *hx, int m, int n, void *adata)
{
	optimzeDataStruct* pData = (optimzeDataStruct*)adata;

	double norm2_x = 0.0;

	int nImages = pData->pBa->m_SensorModleMap.size();
	int nTotalParam = pData->pBa->m_ParamPosMap.rbegin()->second.end;
	int nUnknownCoordinate = pData->pBa->m_nUnknownPoints * 2;

	int nTotalEquation = 0;
	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = pData->GptMapByImages->begin();
		iter != pData->GptMapByImages->end();++iter)
	{
		nTotalEquation += 2 * (int)iter->second.getTiePoints().size();
	}
	// update parameters and coordinates
	for(std::map<int, radiBlockTieGptSet>::iterator iter = pData->GptMapByImages->begin();
		iter != pData->GptMapByImages->end();++iter)
	{
		int imageId = iter->first;
		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
		{
			for(int k = 0;k < static_cast<int>(pData->pBa->m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
			{
				pData->pBa->m_SensorModleMap[imageId]->setAdjustableParameter(k, p[pData->pBa->m_ParamPosMap[imageId].begin + k]);
				if(iter->second.getTiePoints()[j]->m_nUnknownIndex > -1)
				{
					ossimGpt gptUpdate(p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 0],
						p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 1], 0.0);
					gptUpdate.hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(gptUpdate);
					iter->second.refTiePoints()[j]->setGroundPoint(gptUpdate);
				}
			}
		}
	}

	//register int k, k1, i;
	//for(k=0; k<n; ++k){
	//	k1=k+1; // k is zero-based, convert to one-based
	//	i=DIV(k1+1, 2) - 1; // convert i to zero-based
	//	if(k1%2==1) // k1 odd
	//		hx[k]=10.0*(p[i]*p[i]-p[i+1]);
	//	else // k1 even
	//		hx[k]=p[i]-1.0;
	//}

	NEWMAT::ColumnVector newresidue = pData->pBa->getResidue(*pData->GptMapByImages);
	pData->pBa->updateWeightsMatrix(newresidue, pData->mode);

	double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1
	int nEquationIndex = 0;
	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = pData->GptMapByImages->begin();
		iter != pData->GptMapByImages->end();++iter)
	{
		int imageId = iter->first;
		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
		{
			NEWMAT::Matrix A;
			NEWMAT::Matrix B;
			NEWMAT::ColumnVector L;
			// use one gcp to build two error equations
			pData->pBa->m_SensorModleMap[imageId]->buildErrorEquation(*iter->second.getTiePoints()[j], iter->second.getTiePoints()[j]->m_nType, A, B, L, deltap_scale);
			hx[nEquationIndex] = -L.element(0) * sqrt(pData->pBa->m_wgtMatrix.element(nEquationIndex));
			hx[nEquationIndex+1] = -L.element(1) * sqrt(pData->pBa->m_wgtMatrix.element(nEquationIndex+1));
			nEquationIndex += 2;
		}
	}
}

void radiBlockAdjustment::splm_jac(double *p, struct splm_crsm *jac, int m, int n, void *adata)
{
	optimzeDataStruct* pData = (optimzeDataStruct*)adata;

	double norm2_x = 0.0;

	int nImages = pData->pBa->m_SensorModleMap.size();
	int nTotalParam = pData->pBa->m_ParamPosMap.rbegin()->second.end;
	int nUnknownCoordinate = pData->pBa->m_nUnknownPoints * 2;

	int nTotalEquation = 0;
	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = pData->GptMapByImages->begin();
		iter != pData->GptMapByImages->end();++iter)
	{
		nTotalEquation += 2 * (int)iter->second.getTiePoints().size();
	}
	// update parameters and coordinates
	for(std::map<int, radiBlockTieGptSet>::iterator iter = pData->GptMapByImages->begin();
		iter != pData->GptMapByImages->end();++iter)
	{
		int imageId = iter->first;
		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
		{
			for(int k = 0;k < static_cast<int>(pData->pBa->m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
			{
				pData->pBa->m_SensorModleMap[imageId]->setAdjustableParameter(k, p[pData->pBa->m_ParamPosMap[imageId].begin + k]);
				if(iter->second.getTiePoints()[j]->m_nUnknownIndex > -1)
				{
					ossimGpt gptUpdate(p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 0],
						p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 1], 0.0);
					gptUpdate.hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(gptUpdate);
					iter->second.refTiePoints()[j]->setGroundPoint(gptUpdate);
				}
			}
		}
	}

	NEWMAT::ColumnVector newresidue = pData->pBa->getResidue(*pData->GptMapByImages);
	pData->pBa->updateWeightsMatrix(newresidue, pData->mode);

	int iJacobian = 0;
#define STORE_SPLM_JACOBIAN(col, g)                  \
	do                                      \
	{                                       \
	jac->colidx[ iJacobian ] = col;           \
	jac->val[ iJacobian ] = g;             \
	iJacobian++;                          \
	} while(0)

	//register int k, k1, i;
	//int l;

	//for(k=l=0; k<n; ++k){
	//	jac->rowptr[k]=l;
	//	k1=k+1; // k is zero-based, convert to one-based
	//	i=DIV(k1+1, 2) - 1; // convert i to zero-based
	//	if(k1%2==1){ // k1 odd, hx[k]=10*(p[i]*p[i]-p[i+1])
	//		jac->val[l]=20.0*p[i]; jac->colidx[l++]=i;
	//		jac->val[l]=-10.0; jac->colidx[l++]=i+1;
	//	}
	//	else { // k1 even, hx[k]=p[i]-1.0
	//		jac->val[l]=1.0; jac->colidx[l++]=i;
	//	}
	//}
	//jac->rowptr[n]=l;

	double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1
	int nEquationIndex = 0;
	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = pData->GptMapByImages->begin();
		iter != pData->GptMapByImages->end();++iter)
	{
		int imageId = iter->first;
		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
		{
			NEWMAT::Matrix A;
			NEWMAT::Matrix B;
			NEWMAT::ColumnVector L;
			// use one gcp to build two error equations
			radiBlockTieGpt tiePoint = *iter->second.getTiePoints()[j];
			ossimSensorModel* model = pData->pBa->m_SensorModleMap[imageId];
			model->buildErrorEquation(tiePoint, tiePoint.m_nType, A, B, L, deltap_scale);
			// add the two equations to the block adjustment error equations set

			// In this sample problem, every measurement depends on every element of the
			// state vector, so I loop through all the state vectors here. In practice
			// libdogleg is meant to be applied to sparse problems, where this internal
			// loop would be MUCH shorter than Nstate long
			jac->rowptr[nEquationIndex] = iJacobian;
			double val_register;
			for(int k = 0;k < static_cast<int>(pData->pBa->m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
			{
				val_register = pData->pBa->m_wgtMatrix.element(nEquationIndex) * A.element(0, k);
				STORE_SPLM_JACOBIAN( pData->pBa->m_ParamPosMap[imageId].begin + k, val_register );
			}
			if(tiePoint.m_nUnknownIndex > -1)
			{
				for(int kk = 0;kk < 2;++kk)
				{
					val_register = sqrt(pData->pBa->m_wgtMatrix.element(nEquationIndex)) * B.element(0, kk);
					STORE_SPLM_JACOBIAN( nTotalParam + tiePoint.m_nUnknownIndex * 2 + kk, val_register );
				}
			}
			jac->rowptr[nEquationIndex+1] = iJacobian;
			for(int k = 0;k < static_cast<int>(pData->pBa->m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
			{
				val_register = sqrt(pData->pBa->m_wgtMatrix.element(nEquationIndex+1)) * A.element(1, k);
				STORE_SPLM_JACOBIAN( pData->pBa->m_ParamPosMap[imageId].begin + k, val_register );
			}
			if(tiePoint.m_nUnknownIndex > -1)
			{
				for(int kk = 0;kk < 2;++kk)
				{
					val_register = sqrt(pData->pBa->m_wgtMatrix.element(nEquationIndex+1)) * B.element(1, kk);
					STORE_SPLM_JACOBIAN( nTotalParam + tiePoint.m_nUnknownIndex * 2 + kk, val_register );
				}
			}

			nEquationIndex += 2;
		}
	}
	jac->rowptr[nTotalEquation] = iJacobian;
}

void radiBlockAdjustment::sparselm_adjustment(radiBlockTieGptSet& GptList, RobustMode mode/* = NONE*/)
{
	int nImages = m_SensorModleMap.size();
	int nTotalParam = m_ParamPosMap.rbegin()->second.end;
	int nUnknownPoints = initiateGpt(GptList);
	int nUnknownCoordinate = m_nUnknownPoints * 2;

	std::map<int, radiBlockTieGptSet> GptMapByImages = classifyGptByImages( GptList );

	int nTotalEquation = 0;
	for(std::map<int, radiBlockTieGptSet>::iterator iter = GptMapByImages.begin();
		iter != GptMapByImages.end();++iter)
	{
		nTotalEquation += 2 * (int)iter->second.getTiePoints().size();
	}
	m_wgtMatrix.ReSize(nTotalEquation);
	m_wgtMatrix = 1;

	int Nstate = nTotalParam + nUnknownCoordinate;
	int Nmeasurements = nTotalEquation;
	double *p = new double[Nstate];
	for(std::map<int, radiBlockTieGptSet>::iterator iter = GptMapByImages.begin();
		iter != GptMapByImages.end();++iter)
	{
		int imageId = iter->first;
		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
		{
			for(int k = 0;k < static_cast<int>(m_SensorModleMap[imageId]->getNumberOfAdjustableParameters());++k)
			{
				p[m_ParamPosMap[imageId].begin + k] = m_SensorModleMap[imageId]->getAdjustableParameter(k);
				if(iter->second.getTiePoints()[j]->m_nUnknownIndex > -1)
				{
					ossimGpt gpt = iter->second.refTiePoints()[j]->getGroundPoint();
					p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 0] = gpt.lat;
					p[nTotalParam + iter->second.getTiePoints()[j]->m_nUnknownIndex * 2 + 1] = gpt.lon;
				}
			}
		}
	}
	optimzeDataStruct dataStruct;
	dataStruct.pBa = this;
	dataStruct.GptMapByImages = &GptMapByImages;
	dataStruct.mode = mode;
	
	double opts[SPLM_OPTS_SZ], info[SPLM_INFO_SZ];
	int m, n, ret;
	int nnz;
	m = nTotalParam + nUnknownCoordinate;
	n = nTotalEquation;
	nnz = 0;
	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = GptMapByImages.begin();
		iter != GptMapByImages.end();++iter)
	{
		int imageId = iter->first;
		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
		{
			nnz += m_SensorModleMap[imageId]->getNumberOfAdjustableParameters() * 2;
			if (iter->second.getTiePoints()[j]->m_nUnknownIndex > -1)
			{
				nnz += 2 * 2;
			}
		}
	}

	opts[0]=SPLM_INIT_MU; opts[1]=SPLM_STOP_THRESH; opts[2]=SPLM_STOP_THRESH;
	opts[3]=SPLM_STOP_THRESH;
	opts[4]=SPLM_DIFF_DELTA; // relevant only if finite difference approximation to Jacobian is used
	opts[5]=SPLM_CHOLMOD; // use CHOLMOD
	//opts[5]=SPLM_PARDISO; // use PARDISO


	ret = sparselm_dercrs(splm_func, splm_jac, p, NULL, m, 0, n, nnz, -1, 1000, opts, info, &dataStruct); // CRS Jacobian


	printf("sparseLM returned %d in %g iter, reason %g\n ", ret, info[5], info[6]);
	double rmse0 = sqrt(info[0]*2.0/(double)nTotalEquation);
	double rmse1 = sqrt(info[1]*2.0/(double)nTotalEquation);
	printf("Initial RMSE: %g\n Optimized RMSE: %g\n ", rmse0, rmse1);
	//for(i=0; i<m; ++i)
	//	printf("%.7g ", p[i]);
	//printf("\n\nMinimization info:\n");
	//for(i=0; i<SPLM_INFO_SZ; ++i)
	//	printf("%g ", info[i]);
	//printf("\n");

	delete []p;
}

void radiBlockAdjustment::lineSampleToWorld(pair<int, ossimDpt> imagePt, ossimGpt& gpt) const
{
	std::map<int, ossimSensorModel*>::const_iterator modelIter =  m_SensorModleMap.find(imagePt.first);
	if (modelIter != m_SensorModleMap.end())
	{
		modelIter->second->lineSampleToWorld(imagePt.second, gpt);
	}
}

//void radiBlockAdjustment::solve2LeastSquares(NEWMAT::Matrix &N11, NEWMAT::Matrix &N12, NEWMAT::Matrix &N22,
//				   NEWMAT::ColumnVector &L1, NEWMAT::ColumnVector &L2,
//				   NEWMAT::ColumnVector &deltap, NEWMAT::ColumnVector &deltac)
//{
//	NEWMAT::Matrix A;
//	NEWMAT::Matrix r;
//	A = N22 - N12.t() * invert(N11) * N12;
//	r = L2 - N12.t() * invert(N11)*L1;
//
//	double maxdiag=0.0;
//	for(int d=1;d<=A.Nrows();++d) {
//		if (maxdiag < A(d,d)) maxdiag = A(d,d);
//	}
//	double damping = 1e-3 * maxdiag;
//	double olddamping = 0.0;
//	for(int d=1;d <= A.Nrows();++d) A(d,d) += damping - olddamping;
//
//	deltac = invert(A)*r;
//	deltap = invert(N11) * (L1 - N12 * deltac);
//}
///*!
//* solves Ax = r , with A symmetric positive definite
//* A can be rank deficient
//* size of A is typically between 10 and 100 rows
//*/
//NEWMAT::ColumnVector radiBlockAdjustment::solveLeastSquares(NEWMAT::SymmetricMatrix& A,  NEWMAT::ColumnVector& r)const
//{
//	NEWMAT::ColumnVector x = invert(A)*r;
//	return x;
//}
///*!
//* solves Ax = r , with A symmetric positive definite
//* A can be rank deficient
//* size of A is typically between 10 and 100 rows
//*/
////NEWMAT::ColumnVector 
////ossimBlockAdjustment::solveLeastSquares(NEWMAT::Matrix& A,  NEWMAT::ColumnVector& r)const
////{
////	NEWMAT::ColumnVector x = invert(A)*r;
////	return x;
////}
//NEWMAT::Matrix radiBlockAdjustment::invert(const NEWMAT::Matrix& m)const
//{
//	ossim_uint32 idx = 0;
//	NEWMAT::DiagonalMatrix d;
//	NEWMAT::Matrix u;
//	NEWMAT::Matrix v;
//	// decompose m.t*m which is stored in Temp into the singular values and vectors.
//	//
//	NEWMAT::SVD(m, d, u, v, true, true);
//	// invert the diagonal
//	// this is just doing the reciprical fo all diagonal components and store back int
//	// d.  ths compute d inverse.
//	//
//	for(idx=0; idx < (ossim_uint32)d.Ncols(); ++idx)
//	{
//		if(d[idx] > 1e-14) //TBC : use DBL_EPSILON ?
//		{
//			d[idx] = 1.0/d[idx];
//		}
//		else
//		{
//			d[idx] = 0.0;
//			//DEBUG TBR
//			cout<<"warning: singular matrix in SVD"<<endl;
//		}
//	}
//	//compute inverse of decomposed m;
//	return v*d*u.t();
//}

void radiBlockAdjustment::updateSensorModels(NEWMAT::ColumnVector deltap)
{
	for(std::map<int, ossimSensorModel*>::iterator iter = m_SensorModleMap.begin();
		iter != m_SensorModleMap.end();++iter)
	{
		int imageId = iter->first;
		int np = iter->second->getNumberOfAdjustableParameters();
		NEWMAT::ColumnVector cparm(np), nparm(np);
		//get current adjustment (between -1 and 1 normally) and convert to ColumnVector
		ossimAdjustmentInfo cadj;
		iter->second->getAdjustment(cadj);
		std::vector< ossimAdjustableParameterInfo >& parmlist = cadj.getParameterList();
		for(int n=0;n<np;++n)
		{
			cparm(n+1) = parmlist[n].getParameter();
		}
		for(int j = 0;j < np;++j)
		{
			nparm(j + 1) = cparm(j + 1) + deltap(m_ParamPosMap[imageId].begin + j + 1);
		}
		for(int n=0;n<np;++n)
		{
			iter->second->setAdjustableParameter(n, nparm(n+1), false); //do not update now, wait
		}
		iter->second->updateModel();
	}
}

void radiBlockAdjustment::updateCoordinates(vector< vector< radiBlockTieGpt >> &GptListByImages, NEWMAT::ColumnVector deltac)
{
	//int nc = deltac.Nrows() / 3;

	//for(int i = 0;i < static_cast<int>(GptListByImages.size());++i)
	//{
	//	for(int j = 0;j < static_cast<int>(GptListByImages[i].size());++j)
	//	{
	//		if(GptListByImages[i][j].m_nUnknownIndex > -1)
	//		{
	//			ossimGpt gpt;
	//			gpt.lat = GptListByImages[i][j].getGroundPoint().lat + 0.001 * deltac(1 + 3 * GptListByImages[i][j].m_nUnknownIndex + 0);
	//			gpt.lon = GptListByImages[i][j].getGroundPoint().lon + 0.001 * deltac(1 + 3 * GptListByImages[i][j].m_nUnknownIndex + 1);
	//			gpt.hgt = GptListByImages[i][j].getGroundPoint().hgt + deltac(1 + 3 * GptListByImages[i][j].m_nUnknownIndex + 2);
	//			GptListByImages[i][j].setGroundPoint(gpt);
	//		}
	//	}
	//}
}

void radiBlockAdjustment::updateCoordinates(std::map<int, radiBlockTieGptSet>& GptMapByImages, NEWMAT::ColumnVector deltac)
{
	int nc = deltac.Nrows() / 2;

	for(std::map<int, radiBlockTieGptSet>::iterator iter = GptMapByImages.begin();
		iter != GptMapByImages.end();++iter)
	{
		for(int j = 0;j < static_cast<int>(iter->second.refTiePoints().size());++j)
		{
			if(iter->second.refTiePoints()[j]->m_nUnknownIndex > -1)
			{
				ossimGpt gpt;
				gpt.lat = iter->second.refTiePoints()[j]->getGroundPoint().lat + 0.001 * deltac(1 + 2 * iter->second.refTiePoints()[j]->m_nUnknownIndex + 0);
				gpt.lon = iter->second.refTiePoints()[j]->getGroundPoint().lon + 0.001 * deltac(1 + 2 * iter->second.refTiePoints()[j]->m_nUnknownIndex + 1);
				//gpt.hgt = iter->second.refTiePoints()[j]->getGroundPoint().hgt + deltac(1 + 3 * iter->second.refTiePoints()[j]->m_nUnknownIndex + 2);
				gpt.hgt = ossimElevManager::instance()->getHeightAboveEllipsoid(gpt);
				iter->second.refTiePoints()[j]->setGroundPoint(gpt);
			}
		}
	}
}

NEWMAT::ColumnVector radiBlockAdjustment::getResidue(const vector< vector< radiBlockTieGpt >>& GptListByImages)
{
	NEWMAT::ColumnVector residue;
	//int nImages = m_SensorModleList.size();
	//int nTotalEquation = 0;
	//for(int i = 0;i < nImages;++i)
	//{
	//	nTotalEquation += 2 * GptListByImages[i].size();
	//}
	//residue.ReSize(nTotalEquation);
	//residue = 0.0;
	//unsigned long c=1;
	//for(int i = 0;i < nImages;++i)
	//{
	//	for(int j = 0;j < static_cast<int>(GptListByImages[i].size());++j)
	//	{
	//		ossimDpt resIm;
	//		resIm = GptListByImages[i][j].tie - m_SensorModleList[i]->forward(GptListByImages[i][j]);
	//		residue(c++) = resIm.x;
	//		residue(c++) = resIm.y;
	//	}
	//}
	return residue;
}

NEWMAT::ColumnVector radiBlockAdjustment::getResidue(const std::map<int, radiBlockTieGptSet>& GptMapByImages)
{
	NEWMAT::ColumnVector residue;
	int nTotalEquation = 0;
	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = GptMapByImages.begin();
		iter != GptMapByImages.end();++iter)
	{
		nTotalEquation += 2 * (int)iter->second.getTiePoints().size();
	}

	residue.ReSize(nTotalEquation);
	residue = 0.0;
	unsigned long c=1;
	for(std::map<int, radiBlockTieGptSet>::const_iterator iter = GptMapByImages.begin();
		iter != GptMapByImages.end();++iter)
	{
		int imageId = iter->first;
		for(int j = 0;j < static_cast<int>(iter->second.getTiePoints().size());++j)
		{
			ossimDpt resIm;
			resIm = iter->second.getTiePoints()[j]->tie - m_SensorModleMap[imageId]->forward(*iter->second.getTiePoints()[j]);
			residue(c++) = resIm.x;
			residue(c++) = resIm.y;
		}

	}
	return residue;
}

void radiBlockAdjustment::saveBlockGpt(vector<radiBlockTieGpt> gptList, ossimString outFile, ossimMapProjection* transMerc/* = NULL*/)
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

void radiBlockAdjustment::updateWeightsMatrix(const NEWMAT::ColumnVector& newresidue, RobustMode mode/* = NONE*/)
{
	if(RobustMode::HUBER == mode)
		updateWeightsMatrixHuber(newresidue);
	else if (RobustMode::BISQUARE == mode)
	{
		updateWeightsMatrixBisquare(newresidue);
	}
	else;
}

void radiBlockAdjustment::updateWeightsMatrixHuber(const NEWMAT::ColumnVector& newresidue, double k/* = 1.345*/)
{
	int num = m_wgtMatrix.Ncols();
	double theta = newresidue.SumSquare() / num;
	for(int i = 0;i < num;i++)
	{
		double e = fabs(newresidue.element(i));
		if(e <= k*theta)
			m_wgtMatrix.element(i) = 1.0;
		else
			m_wgtMatrix.element(i) = k*theta/(e+DBL_EPSILON);
	}
}

void radiBlockAdjustment::updateWeightsMatrixBisquare(const NEWMAT::ColumnVector& newresidue, double k/* = 4.685*/)
{
	int num = m_wgtMatrix.Ncols();
	double theta = newresidue.SumSquare() / num;
	//double c = sqrt(theta) * coeff;
	for(int i = 0;i < num;i++)
	{
		double e = fabs(newresidue.element(i));
		if(e <= k*theta)
			m_wgtMatrix.element(i) = (1.0 - (e*e/(k*theta*k*theta))) * (1.0 - (e*e/(k*theta*k*theta)));
		else
			m_wgtMatrix.element(i) = 0.0;
	}
}

} // end of namespace ossimplugins