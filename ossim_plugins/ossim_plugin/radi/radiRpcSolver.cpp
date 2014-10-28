//*****************************************************************************
// FILE: ossimRpcModel.h
//
// License:  LGPL
// 
// See LICENSE.txt file in the top level directory for more details.
//
// AUTHOR: Garrett Potts
//
//*****************************************************************************
//  $Id: ossimRpcSolver.cpp 18960 2011-02-25 12:07:18Z gpotts $

#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <iterator>

#include <radi/radiRpcSolver.h>
#include <ossim/projection/ossimRpcModel.h>
#include <ossim/projection/ossimProjection.h>
#include <ossim/matrix/newmatap.h>
#include <ossim/matrix/newmatio.h>
#include <ossim/matrix/newmatnl.h>
#include <ossim/matrix/newmatio.h>
#include <ossim/elevation/ossimElevManager.h>
#include <ossim/support_data/ossimNitfRpcBTag.h>
#include <ossim/imaging/ossimImageGeometry.h>
#include <ossim/base/ossim2dTo2dIdentityTransform.h>


#ifndef _WIN64
#pragma comment(lib, "blas_win32_MT.lib")
#pragma comment(lib, "lapack_win32_MT.lib")
#else
#pragma comment(lib, "blas_win64_MT.lib")
#pragma comment(lib, "lapack_win64_MT.lib")
#endif

#pragma comment(lib, "mlpack.lib")

namespace ossimplugins
{

radiRpcSolver::radiRpcSolver(bool useElevation,
                               bool useHeightAboveMSLFlag)
{
   theUseElevationFlag   = useElevation;
   theHeightAboveMSLFlag = useHeightAboveMSLFlag;
   theXNumCoeffs.resize(20);
   theXDenCoeffs.resize(20);
   theYNumCoeffs.resize(20);
   theYDenCoeffs.resize(20);

   std::fill(theXNumCoeffs.begin(),
             theXNumCoeffs.end(), 0.0);
   std::fill(theXDenCoeffs.begin(),
             theXDenCoeffs.end(), 0.0);
   std::fill(theYNumCoeffs.begin(),
             theYNumCoeffs.end(), 0.0);
   std::fill(theYDenCoeffs.begin(),
             theYDenCoeffs.end(), 0.0);
   theXNumCoeffs[0] = 1.0;
   theXDenCoeffs[0] = 1.0;
   theYNumCoeffs[0] = 1.0;
   theYDenCoeffs[0] = 1.0;
}

void radiRpcSolver::solveCoefficients(const ossimDrect& imageBounds,
                                       ossimProjection* proj,
                                       ossim_uint32 xSamples,
                                       ossim_uint32 ySamples,
                                       bool shiftTo0Flag)
{
   ossimRefPtr<ossimImageGeometry> geom = new ossimImageGeometry();
   geom->setProjection(proj);
   solveCoefficients(imageBounds, geom.get(), xSamples, ySamples, shiftTo0Flag);
}

void radiRpcSolver::solveCoefficients(const ossimDrect& imageBounds,
                                       ossimImageGeometry* geom,
                                       ossim_uint32 xSamples,
                                       ossim_uint32 ySamples,
                                       bool shiftTo0Flag)
{
   std::vector<ossimGpt> theGroundPoints;
   std::vector<ossimDpt> theImagePoints;
   ossim_uint32 x,y;
   ossim_float64 w = imageBounds.width();
   ossim_float64 h = imageBounds.height();
   ossimGpt gpt;
   ossimGpt defaultGround;
   if(ySamples < 1) ySamples = 12;
   if(xSamples < 1) xSamples = 12;
   srand(time(0));
   double xnorm;
   double ynorm;
   ossimDpt ul = imageBounds.ul();
   ossimDpt shiftTo0(-ul.x,
                     -ul.y);
   for(y = 0; y < ySamples; ++y)
   {
      for(x = 0; x < xSamples; ++x)
      {
         if(ySamples > 1)
         {
//            ynorm = (double)y/(double)(ySamples-1.0);
            ynorm = (double)y/(double)(ySamples);
         }
         else
         {
            ynorm = 0.0;
         }
         if(xSamples > 1)
         {
//            xnorm = (double)x/(double)(xSamples-1.0);
            xnorm = (double)x/(double)(xSamples);
         }
         else
         {
            xnorm = 0.0;
         }
         
//          ossimDpt dpt((.25 + .5*xnorm)*w + ul.x,
//                       (.25 + .5*ynorm)*h + ul.y);
         ossimDpt dpt(w*xnorm + ul.x,
                      h*ynorm + ul.y);
         
         geom->localToWorld(dpt, gpt);
         gpt.changeDatum(defaultGround.datum());

         if(shiftTo0Flag)
         {
            theImagePoints.push_back(dpt+shiftTo0);
         }
         else
         {
            theImagePoints.push_back(dpt);
         }
         if(theHeightAboveMSLFlag)
         {
            double h = ossimElevManager::instance()->getHeightAboveMSL(gpt);
            if(ossim::isnan(h) == false)
            {
               gpt.height(h);
            }
         }
         if(gpt.isHgtNan())
         {
            gpt.height(0.0);
         }
         theGroundPoints.push_back(gpt);
      }
   }
   solveCoefficients(theImagePoints,
                     theGroundPoints);
}

ossimImageGeometry* radiRpcSolver::createRpcModel()const
{
   ossimRpcModel* model = new ossimRpcModel;
   
   model->setAttributes(theImageOffset.x,
                        theImageOffset.y,
                        theImageScale.x,
                        theImageScale.y,
                        theGroundOffset.latd(),
                        theGroundOffset.lond(),
                        theGroundOffset.height(),
                        theLatScale,
                        theLonScale,
                        theHeightScale,
                        theXNumCoeffs,
                        theXDenCoeffs,
                        theYNumCoeffs,
                        theYDenCoeffs);
   return new ossimImageGeometry(new ossim2dTo2dIdentityTransform, model);
}

ossimImageGeometry* radiRpcSolver::createRpcProjection()const
{
   ossimRpcProjection* proj = new ossimRpcProjection;
   
   proj->setAttributes(theImageOffset.x,
                       theImageOffset.y,
                       theImageScale.x,
                       theImageScale.y,
                       theGroundOffset.latd(),
                       theGroundOffset.lond(),
                       theGroundOffset.height(),
                       theLatScale,
                       theLonScale,
                       theHeightScale,
                       theXNumCoeffs,
                       theXDenCoeffs,
                       theYNumCoeffs,
                       theYDenCoeffs);
   return new ossimImageGeometry(new ossim2dTo2dIdentityTransform, proj);
}

const std::vector<double>& radiRpcSolver::getImageXNumCoefficients()const
{
   return theXNumCoeffs;
}

const std::vector<double>& radiRpcSolver::getImageXDenCoefficients()const
{
   return theXDenCoeffs;
}

const std::vector<double>& radiRpcSolver::getImageYNumCoefficients()const
{
   return theYNumCoeffs;
}

const std::vector<double>& radiRpcSolver::getImageYDenCoefficients()const
{
   return theYDenCoeffs;
}

double radiRpcSolver::getImageXOffset()const
{
   return theImageOffset.x;
}

double radiRpcSolver::getImageYOffset()const
{
   return theImageOffset.y;
}

double radiRpcSolver::getLatOffset()const
{
   return theGroundOffset.latd();
}

double radiRpcSolver::getLonOffset()const
{
   return theGroundOffset.lond();
}

double radiRpcSolver::getHeightOffset()const
{
   return theGroundOffset.height();
}

double radiRpcSolver::getImageXScale()const
{
   return theImageScale.x;
}

double radiRpcSolver::getImageYScale()const
{
   return theImageScale.y;
}

double radiRpcSolver::getLatScale()const
{
   return theLatScale;
}

double radiRpcSolver::getLonScale()const
{
   return theLonScale;
}

double radiRpcSolver::getHeightScale()const
{
   return theHeightScale;
}

double radiRpcSolver::getRmsError()const
{
   return theError;
}

void radiRpcSolver::solveInitialCoefficients(arma::vec& coeff,
                                              const std::vector<double>& f,
                                              const std::vector<double>& x,
                                              const std::vector<double>& y,
                                              const std::vector<double>& z)const
{
   ossim_uint32 idx = 0;
   arma::mat m;
   arma::vec r((int)f.size());
   for(idx = 0; idx < f.size(); ++idx)
   {
      r[idx] = f[idx];
   }
   setupSystemOfEquations(m,
                          r,
                          x,
                          y,
                          z);
   
   coeff = arma::inv(m.t()*m)*m.t()*r;
}


void radiRpcSolver::solveCoefficients(arma::vec& coeff,
									   const std::vector<double>& f,
									   const std::vector<double>& x,
									   const std::vector<double>& y,
									   const std::vector<double>& z)const
{
	// this is an iterative  linear least square fit.  We really pobably need
	// a nonlinear fit instead
	//
	ossim_uint32 idx = 0;
	arma::mat m;
	int num = (int)f.size();
	arma::vec r;
	r.resize(num);
	for(idx = 0; idx < num; ++idx)
	{
		r[idx] = f[idx];
	}
	setupSystemOfEquations(m,
		r,
		x,
		y,
		z);

	////int nCols = m.n_cols;
	////int nRows = m.n_rows;
	////arma::mat matX(nRows, nCols);
	////arma::vec matY(nRows);

	////for (int i = 0;i < nRows;++i)
	////{
	////	matY[i] = r(i+1);
	////	for (int j = 0;j < nCols;++j)
	////	{
	////		matX(i,j) = m(i+1,j+1);
	////	}
	////}

	arma::mat matX = m;
	arma::vec matY = r;
	// Make sure y is oriented the right way.
	//cout<<matY.n_rows<<endl;
	if (matY.n_rows == 1)
		matY = trans(matY);
	if (matY.n_cols > 1)
		Log::Fatal << "Only one column or row allowed in responses file!" << endl;

	if (matY.n_elem != matX.n_rows)
		Log::Fatal << "Number of responses must be equal to number of rows of X!"
		<< endl;

	double lambda1 = 2e-5;
	double lambda2 = 0.0;

	//arma::vec sortedAbsCorr = sort(abs(matX.t() * matY));
	//double lambda1 = sortedAbsCorr(matX.n_cols / 2);
	//double lambda2 = 0.0;
	// Do LARS.
	LARS lars(false, lambda1, lambda2);
	//vec beta;
	lars.Regress(matX, matY, coeff, false /* do not transpose */);

	//coeff.resize(beta.n_elem);
	//for(int i = 0;i < beta.n_elem;i++)
	//{
	//	coeff[i] = beta[i];
	//}
	
	//int nCols = m.n_cols;
	//arma::mat lambda = arma::eye(nCols, nCols);
	//lambda = lambda * 1.0e-6;
	//coeff = arma::inv(m.t()*m+lambda)*m.t()*r;

	//fstream fs;
	//fs.open("r.txt", ios_base::out);
	//fs<<r<<endl;
	//fs.close();


	//fs.open("m.txt", ios_base::out);
	//fs<<m<<endl;
	//fs.close();


	//fs.open("coeff.txt", ios_base::out);
	//fs<<coeff<<endl;
	//fs.close();
}

void radiRpcSolver::solveAllCoefficients(arma::vec& coeff,
										 const std::vector<double>& f1,
										 const std::vector<double>& f2,
									  const std::vector<double>& x,
									  const std::vector<double>& y,
									  const std::vector<double>& z,
									  EstimationMethodIndex method/* = LASSO*/,
									  double parameter/* = 1e-5*/)const
{
	// this is an iterative  linear least square fit.  We really pobably need
	// a nonlinear fit instead
	//
	ossim_uint32 idx = 0;
	int num = (int)f1.size();

	arma::vec r1(num);
	for(idx = 0; idx < num; ++idx)
	{
		r1[idx] = f1[idx];
	}

	arma::mat m11;
	setupSystemOfEquations(m11,
		r1,
		x,
		y,
		z);
	arma::vec r2(num);
	for(idx = 0; idx < num; ++idx)
	{
		r2[idx] = f2[idx];
	}
	arma::mat m22;
	setupSystemOfEquations(m22,
		r2,
		x,
		y,
		z);

	arma::vec matY = arma::join_vert( r1, r2 );

	arma::mat zeros_block = arma::zeros(num, 39);
	arma::mat m1 = arma::join_horiz(m11, zeros_block);
	arma::mat m2 = arma::join_horiz( zeros_block, m22);
	arma::mat matX = arma::join_vert(m1, m2);

	////int nCols = m.n_cols;
	////int nRows = m.n_rows;
	////arma::mat matX(nRows, nCols);
	////arma::vec matY(nRows);

	////for (int i = 0;i < nRows;++i)
	////{
	////	matY[i] = r(i+1);
	////	for (int j = 0;j < nCols;++j)
	////	{
	////		matX(i,j) = m(i+1,j+1);
	////	}
	////}

	//arma::mat matX = m;
	//arma::vec matY = r;
	// Make sure y is oriented the right way.
	//cout<<matY.n_rows<<endl;
	if (matY.n_rows == 1)
		matY = trans(matY);
	if (matY.n_cols > 1)
		Log::Fatal << "Only one column or row allowed in responses file!" << endl;

	if (matY.n_elem != matX.n_rows)
		Log::Fatal << "Number of responses must be equal to number of rows of X!"
		<< endl;


	//arma::vec sortedAbsCorr = sort(abs(matX.t() * matY));
	//double lambda1 = sortedAbsCorr(matX.n_cols / 2);
	//double lambda2 = 0.0;
	if (LASSO == method)
	{
		// Do LARS.
		//double lambda1 = 1e-5;
		//double lambda2 = 0.0;
		LARS lars(false, parameter, 0.0);
		//vec beta;
		lars.Regress(matX, matY, coeff, false /* do not transpose */);
	}
	else if (RIDGE == method)
	{
		arma::mat A = matX.t()*matX;
		arma::mat lambda = arma::eye(A.n_rows, A.n_cols) * parameter;
		coeff = arma::inv(A+lambda)*matX.t()*matY;
	}
	else if (LS == method)
	{
		coeff = arma::inv(matX.t()*matX)*matX.t()*matY;
	}

	//coeff.resize(beta.n_elem);
	//for(int i = 0;i < beta.n_elem;i++)
	//{
	//	coeff[i] = beta[i];
	//}

	//int nCols = m.n_cols;
	//arma::mat lambda = arma::eye(nCols, nCols);
	//lambda = lambda * 1.0e-6;
	//coeff = arma::inv(m.t()*m+lambda)*m.t()*r;

	//fstream fs;
	//fs.open("matY.txt", ios_base::out);
	//fs<<matY<<endl;
	//fs.close();


	//fs.open("matX.txt", ios_base::out);
	//fs<<matX<<endl;
	//fs.close();


	//fs.open("coeff.txt", ios_base::out);
	//fs<<coeff<<endl;
	//fs.close();
}


void radiRpcSolver::setupSystemOfEquations(arma::mat& equations,
                                            const arma::vec& f,
                                            const std::vector<double>& x,
                                            const std::vector<double>& y,
                                            const std::vector<double>& z)const
{
   ossim_uint32 idx;
   equations.resize(f.n_rows, 39);
   
   for(idx = 0; idx < (ossim_uint32)f.n_rows;++idx)
   {
	   equations(idx, 0) = 1;
      equations(idx,1)  = x[idx];
      equations(idx,2)  = y[idx];
      equations(idx,3)  = z[idx];
      equations(idx,4)  = x[idx]*y[idx];
      equations(idx,5)  = x[idx]*z[idx];
      equations(idx,6)  = y[idx]*z[idx];
      equations(idx,7)  = x[idx]*x[idx];
      equations(idx,8)  = y[idx]*y[idx];
      equations(idx,9)  = z[idx]*z[idx];
      equations(idx,10) = x[idx]*y[idx]*z[idx];
      equations(idx,11) = x[idx]*x[idx]*x[idx];
      equations(idx,12) = x[idx]*y[idx]*y[idx];
      equations(idx,13) = x[idx]*z[idx]*z[idx];
      equations(idx,14) = x[idx]*x[idx]*y[idx];
      equations(idx,15) = y[idx]*y[idx]*y[idx];
      equations(idx,16) = y[idx]*z[idx]*z[idx];
      equations(idx,17) = x[idx]*x[idx]*z[idx];
      equations(idx,18) = y[idx]*y[idx]*z[idx];
      equations(idx,19) = z[idx]*z[idx]*z[idx];
      equations(idx,20) = -f[idx]*x[idx];
      equations(idx,21) = -f[idx]*y[idx];
      equations(idx,22) = -f[idx]*z[idx];
      equations(idx,23) = -f[idx]*x[idx]*y[idx];
      equations(idx,24) = -f[idx]*x[idx]*z[idx];
      equations(idx,25) = -f[idx]*y[idx]*z[idx];
      equations(idx,26) = -f[idx]*x[idx]*x[idx];
      equations(idx,27) = -f[idx]*y[idx]*y[idx];
      equations(idx,28) = -f[idx]*z[idx]*z[idx];
      equations(idx,29) = -f[idx]*x[idx]*y[idx]*z[idx];
      equations(idx,30) = -f[idx]*x[idx]*x[idx]*x[idx];
      equations(idx,31) = -f[idx]*x[idx]*y[idx]*y[idx];
      equations(idx,32) = -f[idx]*x[idx]*z[idx]*z[idx];
      equations(idx,33) = -f[idx]*x[idx]*x[idx]*y[idx];
      equations(idx,34) = -f[idx]*y[idx]*y[idx]*y[idx];
      equations(idx,35) = -f[idx]*y[idx]*z[idx]*z[idx];
      equations(idx,36) = -f[idx]*x[idx]*x[idx]*z[idx];
      equations(idx,37) = -f[idx]*y[idx]*y[idx]*z[idx];
      equations(idx,38) = -f[idx]*z[idx]*z[idx]*z[idx];
   }
}

void radiRpcSolver::setupWeightMatrix(NEWMAT::DiagonalMatrix& result, // holds the resulting weights
                                       const NEWMAT::ColumnVector& coefficients,
                                       const NEWMAT::ColumnVector& f,
                                       const std::vector<double>& x,
                                       const std::vector<double>& y,
                                       const std::vector<double>& z)const
{
   result.ReSize(f.Nrows());
   ossim_uint32 idx = 0;
   ossim_uint32 idx2 = 0;
   NEWMAT::RowVector row(coefficients.Nrows());
   
    for(idx = 0; idx < (ossim_uint32)f.Nrows(); ++idx)
    {
       row[0]  = 1;
       row[1]  = x[idx];
       row[2]  = y[idx];
       row[3]  = z[idx];
       row[4]  = x[idx]*y[idx];
       row[5]  = x[idx]*z[idx];
       row[6]  = y[idx]*z[idx];
       row[7]  = x[idx]*x[idx];
       row[8]  = y[idx]*y[idx];
       row[9]  = z[idx]*z[idx];
       row[10] = x[idx]*y[idx]*z[idx];
       row[11] = x[idx]*x[idx]*x[idx];
       row[12] = x[idx]*y[idx]*y[idx];
       row[13] = x[idx]*z[idx]*z[idx];
       row[14] = x[idx]*x[idx]*y[idx];
       row[15] = y[idx]*y[idx]*y[idx];
       row[16] = y[idx]*z[idx]*z[idx];
       row[17] = x[idx]*x[idx]*z[idx];
       row[18] = y[idx]*y[idx]*z[idx];
       row[19] = z[idx]*z[idx]*z[idx];

      result[idx] = 0.0;
      for(idx2 = 0; idx2 < (ossim_uint32)row.Ncols(); ++idx2)
      {
         result[idx] += row[idx2]*coefficients[idx2];
      }

      if(result[idx] > FLT_EPSILON)
      {
         result[idx] = 1.0/result[idx];
      }
    }
}

double radiRpcSolver::eval(const std::vector<double>& coeff,
                            double x,
                            double y,
                            double z)const
{
   return coeff[ 0]       + coeff[ 1]*x     + coeff[ 2]*y     + coeff[ 3]*z     +
          coeff[ 4]*x*y   + coeff[ 5]*x*z   + coeff[ 6]*y*z   + coeff[ 7]*x*x   +
          coeff[ 8]*y*y   + coeff[ 9]*z*z   + coeff[10]*x*y*z + coeff[11]*x*x*x +
          coeff[12]*x*y*y + coeff[13]*x*z*z + coeff[14]*x*x*y + coeff[15]*y*y*y +
          coeff[16]*y*z*z + coeff[17]*x*x*z + coeff[18]*y*y*z + coeff[19]*z*z*z;
}


ossimRefPtr<ossimNitfRegisteredTag> radiRpcSolver::getNitfRpcBTag() const
{
   ossimNitfRpcBTag* rpcbTag = new ossimNitfRpcBTag();

   // success always true
   rpcbTag->setSuccess(true);

   // temp "0"...
   rpcbTag->setErrorBias(0.0);

   // temp "0"...
   rpcbTag->setErrorRand(0.0);

   // line offset
   rpcbTag->setLineOffset(static_cast<ossim_uint32>(getImageYOffset()));

   // sample offset
   rpcbTag->setSampleOffset(static_cast<ossim_uint32>(getImageXOffset()));

   // latitude offset
   rpcbTag->setGeodeticLatOffset(getLatOffset());

   // longitude offset
   rpcbTag->setGeodeticLonOffset(getLonOffset());

   // height offset
   rpcbTag->setGeodeticHeightOffset(
      static_cast<ossim_int32>(getHeightOffset()));

   // line scale
   rpcbTag->setLineScale(static_cast<ossim_uint32>(getImageYScale()));

   // sample scale
   rpcbTag->setSampleScale(static_cast<ossim_uint32>(getImageXScale()));

   // latitude scale
   rpcbTag->setGeodeticLatScale(getLatScale());

   // longitude scale
   rpcbTag->setGeodeticLonScale(getLonScale());

   // height scale
   rpcbTag->setGeodeticHeightScale(static_cast<ossim_int32>(getHeightScale()));

   // line numerator coefficients
   rpcbTag->setLineNumeratorCoeff(getImageYNumCoefficients());
   
   // line denominator coefficients
   rpcbTag->setLineDenominatorCoeff(getImageYDenCoefficients());

   // sample numerator coefficients
   rpcbTag->setSampleNumeratorCoeff(getImageXNumCoefficients());

   // sample denominator coefficients
   rpcbTag->setSampleDenominatorCoeff(getImageXDenCoefficients());

   // Return it as an ossimRefPtr<ossimNitfRegisteredTag>...
   ossimRefPtr<ossimNitfRegisteredTag> tag = rpcbTag;
   
   return tag;
}


void radiRpcSolver::update_normalization_params(const std::vector<ossimDpt>& imagePoints,
								   const std::vector<ossimGpt>& groundControlPoints)
{
	// compute the image bounds for the given image points
	//

	double latSum=0.0;
	double lonSum=0.0;
	double heightSum=0.0;
	double sampSum = 0.0;
	double lineSum = 0.0;

	ossim_float64 minLat = DBL_MAX;
	ossim_float64 minLon = DBL_MAX;
	ossim_float64 minHgt = DBL_MAX;
	ossim_float64 minSamp = DBL_MAX;
	ossim_float64 minLine = DBL_MAX;

	ossim_float64 maxLat = -1e10;
	ossim_float64 maxLon = -1e10;
	ossim_float64 maxHgt = -1e10;
	ossim_float64 maxSamp = -1e10;
	ossim_float64 maxLine = -1e10;

	for(unsigned int c = 0; c < groundControlPoints.size();++c)
	{
		if(ossim::isnan(groundControlPoints[c].latd()) == false)
		{
			if (minLat > groundControlPoints[c].latd())
			{
				minLat = groundControlPoints[c].latd();
			}
			if (maxLat < groundControlPoints[c].latd())
			{
				maxLat = groundControlPoints[c].latd();
			}
			latSum += groundControlPoints[c].latd();
		}
		if(ossim::isnan(groundControlPoints[c].lond()) == false)
		{
			if (minLon > groundControlPoints[c].lond())
			{
				minLon = groundControlPoints[c].lond();
			}
			if (maxLon < groundControlPoints[c].lond())
			{
				maxLon = groundControlPoints[c].lond();
			}
			lonSum += groundControlPoints[c].lond();
		}
		if(!groundControlPoints[c].isHgtNan() && theUseElevationFlag)
		{
			if (minHgt > groundControlPoints[c].height())
			{
				minHgt = groundControlPoints[c].height();
			}
			if (maxHgt < groundControlPoints[c].height())
			{
				maxHgt = groundControlPoints[c].height();
			}
			heightSum += groundControlPoints[c].height();
		}
	}

	for(unsigned int c = 0; c < imagePoints.size();++c)
	{
		if(ossim::isnan(imagePoints[c].samp) == false)
		{
			if (minSamp > imagePoints[c].samp)
			{
				minSamp = imagePoints[c].samp;
			}
			if (maxSamp < imagePoints[c].samp)
			{
				maxSamp = imagePoints[c].samp;
			}			
			sampSum += imagePoints[c].samp;
		}
		if(ossim::isnan(imagePoints[c].line) == false)
		{
			if (minLine > imagePoints[c].line)
			{
				minLine = imagePoints[c].line;
			}
			if (maxLine < imagePoints[c].line)
			{
				maxLine = imagePoints[c].line;
			}
			lineSum += imagePoints[c].line;
		}
	}
	// set the center ground for the offset
	//
	theGroundOffset = ossimGpt(latSum/groundControlPoints.size(),
		lonSum/groundControlPoints.size(),
		heightSum/groundControlPoints.size());

	theImageOffset = ossimDpt(sampSum/imagePoints.size(), lineSum/imagePoints.size());
	double sampScale = max(fabs(maxSamp - theImageOffset.samp), fabs(theImageOffset.samp - minSamp));
	double lineScale = max(fabs(maxLine - theImageOffset.line), fabs(theImageOffset.line - minLine));
	theImageScale = ossimDpt(sampScale, lineScale);

	theLonScale = max(fabs(maxLon - theGroundOffset.lond()), fabs(theGroundOffset.lond() - minLon));
	theLatScale = max(fabs(maxLat - theGroundOffset.latd()), fabs(theGroundOffset.latd() - minLat));
	theHeightScale = max(fabs(maxHgt - theGroundOffset.height()), fabs(theGroundOffset.height() - minHgt));
}

void radiRpcSolver::normalize(ossimGpt &gpt)
{
	gpt.lat = (gpt.lat - theGroundOffset.latd()) / theLatScale;
	gpt.lon = (gpt.lon - theGroundOffset.lond()) / theLonScale;
	gpt.hgt = (gpt.height() - theGroundOffset.height()) / theHeightScale;
}

void radiRpcSolver::normalize(ossimDpt &dpt)
{
	dpt.samp = (dpt.samp - theImageOffset.samp) / theImageScale.samp;
	dpt.line = (dpt.line - theImageOffset.line) / theImageScale.line;
}

void radiRpcSolver::denormalize(ossimGpt &gpt)
{
	gpt.lat = gpt.lat * theLatScale + theGroundOffset.latd();
	gpt.lon = gpt.lon * theLonScale + theGroundOffset.lond();
	gpt.hgt = gpt.hgt * theHeightScale + theGroundOffset.height();
}

void radiRpcSolver::denormalize(ossimDpt &dpt)
{
	dpt.samp = dpt.samp * theImageScale.samp + theImageOffset.samp;
	dpt.line = dpt.line *  theImageScale.line + theImageOffset.line;
}
//
//void radiRpcSolver::solveCoefficients(const std::vector<ossimDpt>& imagePoints,
//									   const std::vector<ossimGpt>& groundControlPoints,
//									   const ossimDpt& /* imageShift */)
//{
//	if((imagePoints.size() != groundControlPoints.size()))
//	{
//		return;
//	}
//
//	// we will first create f which holds the result of f(x,y,z).
//	// This basically holds the cooresponding image point for each
//	// ground control point.  One for x and a second array for y
//	//
//	std::vector<double> f[2];
//
//	//  Holds the x, y, z vectors
//	//
//	std::vector<double> x;
//	std::vector<double> y;
//	std::vector<double> z;
//	ossim_uint32 c = 0;
//	f[0].resize(imagePoints.size());
//	f[1].resize(imagePoints.size());
//	x.resize(imagePoints.size());
//	y.resize(imagePoints.size());
//	z.resize(imagePoints.size());
//
//	update_normalization_params(imagePoints, groundControlPoints);
//	for(size_t c = 0; c < groundControlPoints.size(); ++c)
//	{
//		ossimGpt gpt = groundControlPoints[c];
//		ossimDpt dpt = imagePoints[c];
//		normalize(gpt);
//		normalize(dpt);
//		f[0][c] = dpt.x;
//		f[1][c] = dpt.y;
//
//		x[c] = gpt.lon;
//		y[c] = gpt.lat;
//		z[c] = gpt.hgt;
//	}
//		
//	bool elevationEnabled = theUseElevationFlag;
//	
//	// now lets solve the coefficients
//	//
//	std::vector<double> coeffx;
//	std::vector<double> coeffy;
//
//	arma::vec coeffxVec;
//	arma::vec coeffyVec;
//	// perform a least squares fit for sample values found in f
//	// given the world values with variables x, y, z
//	//
//	solveCoefficients(coeffxVec,
//		f[0],
//		x,
//		y,
//		z);
//
//
//	// perform a least squares fit for line values found in f
//	// given the world values with variables x, y, z
//	//
//	solveCoefficients(coeffyVec,
//		f[1],
//		x,
//		y,
//		z);
//
//	coeffx.resize(coeffxVec.n_rows);
//	coeffy.resize(coeffyVec.n_rows);
//
//	for(c = 0; c < coeffx.size();++c)
//	{
//		coeffx[c] = coeffxVec[c];
//		coeffy[c] = coeffyVec[c];
//	}
//	// there are 20 numerator coefficients
//	// and 19 denominator coefficients
//	// I believe that the very first one for the
//	// denominator coefficients is fixed at 1.0
//	//
//	std::copy(coeffx.begin(),
//		coeffx.begin()+20,
//		theXNumCoeffs.begin());
//	std::copy(coeffx.begin()+20,
//		coeffx.begin()+39,
//		theXDenCoeffs.begin()+1);
//	std::copy(coeffy.begin(),
//		coeffy.begin()+20,
//		theYNumCoeffs.begin());
//	std::copy(coeffy.begin()+20,
//		coeffy.begin()+39,
//		theYDenCoeffs.begin()+1);
//	theXDenCoeffs[0] = 1.0;
//	theYDenCoeffs[0] = 1.0;
//
//
//	// now lets compute the RMSE for the given control points by feeding it
//	// back through the modeled RPC
//	//
//	ossim_float64  sumSquareError = 0.0;
//	ossim_uint32 idx = 0;
//
//	//    std::cout << "ground offset height = " << theGroundOffset.height()
//	//              << "Height scale         = " << theHeightScale << std::endl;
//	for (idx = 0; idx<imagePoints.size(); idx++)
//	{
//		ossimDpt dpt = imagePoints[idx];
//		ossimGpt gpt = groundControlPoints[idx];
//		normalize(dpt);
//		normalize(gpt);
//		ossim_float64 x = gpt.lon;
//		ossim_float64 y = gpt.lat;
//		ossim_float64 z = gpt.hgt;
//
//		if(ossim::isnan(z))
//		{
//			z = 0.0;
//		}
//		else
//		{
//			z = (z - theGroundOffset.height()/theHeightScale);
//		}
//		ossim_float64 imageX = ((eval(theXNumCoeffs, x, y, z)/
//			eval(theXDenCoeffs, x, y, z))*theImageScale.x) + theImageOffset.x;
//
//		ossim_float64 imageY = ((eval(theYNumCoeffs, x, y, z)/
//			eval(theYDenCoeffs, x, y, z))*theImageScale.y) + theImageOffset.y;
//
//		ossimDpt evalPt(imageX, imageY);
//		ossim_float64 len = (evalPt - imagePoints[idx]).length();
//
//		sumSquareError += (len*len);
//	}
//
//	// set the error
//	//
//	theError = sqrt(sumSquareError/imagePoints.size());
//}

void radiRpcSolver::solveCoefficients(const std::vector<ossimDpt>& imagePoints,
									  const std::vector<ossimGpt>& groundControlPoints,
									  bool b_update_normalization_params,
									  EstimationMethodIndex method/* = LASSO*/,
									  double parameter/* = 1e-5*/,
									  const ossimDpt& /* imageShift */)
{
	if((imagePoints.size() != groundControlPoints.size()))
	{
		return;
	}

	// we will first create f which holds the result of f(x,y,z).
	// This basically holds the cooresponding image point for each
	// ground control point.  One for x and a second array for y
	//
	std::vector<double> f[2];

	//  Holds the x, y, z vectors
	//
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
	ossim_uint32 c = 0;
	f[0].resize(imagePoints.size());
	f[1].resize(imagePoints.size());
	x.resize(imagePoints.size());
	y.resize(imagePoints.size());
	z.resize(imagePoints.size());

	if (b_update_normalization_params)
	{
		update_normalization_params(imagePoints, groundControlPoints);
	}
	for(size_t c = 0; c < groundControlPoints.size(); ++c)
	{
		ossimGpt gpt = groundControlPoints[c];
		ossimDpt dpt = imagePoints[c];
		normalize(gpt);
		normalize(dpt);
		f[0][c] = dpt.x;
		f[1][c] = dpt.y;

		x[c] = gpt.lon;
		y[c] = gpt.lat;
		z[c] = gpt.hgt;
	}

	bool elevationEnabled = theUseElevationFlag;

	// now lets solve the coefficients
	//

	arma::vec coeffVec;
	solveAllCoefficients(coeffVec,
		f[0],
		f[1],
		x,
		y,
		z,
		method,
		parameter);

	std::vector<double> coeff(coeffVec.n_rows);

	for(c = 0; c < coeff.size();++c)
	{
		coeff[c] = coeffVec[c];
	}
	// there are 20 numerator coefficients
	// and 19 denominator coefficients
	// I believe that the very first one for the
	// denominator coefficients is fixed at 1.0
	//
	std::copy(coeff.begin(),
		coeff.begin()+20,
		theXNumCoeffs.begin());
	std::copy(coeff.begin()+20,
		coeff.begin()+39,
		theXDenCoeffs.begin()+1);
	std::copy(coeff.begin()+39,
		coeff.begin()+59,
		theYNumCoeffs.begin());
	std::copy(coeff.begin()+59,
		coeff.begin()+78,
		theYDenCoeffs.begin()+1);
	theXDenCoeffs[0] = 1.0;
	theYDenCoeffs[0] = 1.0;


	// now lets compute the RMSE for the given control points by feeding it
	// back through the modeled RPC
	//
	ossim_float64  sumSquareError = 0.0;
	ossim_uint32 idx = 0;

	//    std::cout << "ground offset height = " << theGroundOffset.height()
	//              << "Height scale         = " << theHeightScale << std::endl;
	for (idx = 0; idx<imagePoints.size(); idx++)
	{
		ossimDpt dpt = imagePoints[idx];
		ossimGpt gpt = groundControlPoints[idx];
		normalize(dpt);
		normalize(gpt);
		ossim_float64 x = gpt.lon;
		ossim_float64 y = gpt.lat;
		ossim_float64 z = gpt.hgt;

		if(ossim::isnan(z))
		{
			z = 0.0;
		}
		else
		{
			z = (z - theGroundOffset.height()/theHeightScale);
		}
		ossim_float64 imageX = ((eval(theXNumCoeffs, x, y, z)/
			eval(theXDenCoeffs, x, y, z))*theImageScale.x) + theImageOffset.x;

		ossim_float64 imageY = ((eval(theYNumCoeffs, x, y, z)/
			eval(theYDenCoeffs, x, y, z))*theImageScale.y) + theImageOffset.y;

		ossimDpt evalPt(imageX, imageY);
		ossim_float64 len = (evalPt - imagePoints[idx]).length();

		sumSquareError += (len*len);
	}

	// set the error
	//
	theError = sqrt(sumSquareError/imagePoints.size());
}
}