#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <ossim/projection/ossimRpcSolver.h>
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
#include <mlpack/methods/lars/lars.hpp>
ossimRpcSolver::ossimRpcSolver(bool useElevation,
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
void ossimRpcSolver::solveCoefficients(const ossimDrect& imageBounds,
                                       ossimProjection* proj,
                                       ossim_uint32 xSamples,
                                       ossim_uint32 ySamples,
                                       bool shiftTo0Flag)
{
   ossimRefPtr<ossimImageGeometry> geom = new ossimImageGeometry();
   geom->setProjection(proj);
   solveCoefficients(imageBounds, geom.get(), xSamples, ySamples, shiftTo0Flag);
}
void ossimRpcSolver::solveCoefficients(const ossimDrect& imageBounds,
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
            ynorm = (double)y/(double)(ySamples - 1);
         }
         else
         {
            ynorm = 0.0;
         }
         if(xSamples > 1)
         {
            xnorm = (double)x/(double)(xSamples - 1);
         }
         else
         {
            xnorm = 0.0;
         }
         
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

//void ossimRpcSolver::solveCoefficients(const std::vector<ossimDpt>& imagePoints,
//									  const std::vector<ossimGpt>& groundControlPoints,
//									  const ossimDpt& imageShift)
//{
//	if((imagePoints.size() != groundControlPoints.size()))
//	{
//		return;
//	}
//	std::vector<double> f[2];
//	std::vector<double> x;
//	std::vector<double> y;
//	std::vector<double> z;
//	ossim_uint32 c = 0;
//	f[0].resize(imagePoints.size());
//	f[1].resize(imagePoints.size());
//	x.resize(imagePoints.size());
//	y.resize(imagePoints.size());
//	z.resize(imagePoints.size());
//	ossimDrect rect(imagePoints);
//
//	double xSum = 0.0;
//	double ySum = 0.0;
//	double latSum=0.0;
//	double lonSum=0.0;
//	double heightSum=0.0;
//	for(c = 0; c < groundControlPoints.size();++c)
//	{
//		if(ossim::isnan(imagePoints[c].x) == false)
//		{
//			xSum += imagePoints[c].x;
//		}
//		if(ossim::isnan(imagePoints[c].y) == false)
//		{
//			ySum += imagePoints[c].y;
//		}
//		if(ossim::isnan(groundControlPoints[c].latd()) == false)
//		{
//			latSum += groundControlPoints[c].latd();
//		}
//		if(ossim::isnan(groundControlPoints[c].lond()) == false)
//		{
//			lonSum += groundControlPoints[c].lond();
//		}
//		if(!groundControlPoints[c].isHgtNan())
//		{
//			if(theUseElevationFlag)
//			{
//				heightSum += groundControlPoints[c].height();
//			}
//		}
//	}
//	ossimGpt centerGround(latSum/groundControlPoints.size(),
//		lonSum/groundControlPoints.size(),
//		heightSum/groundControlPoints.size());
//	ossimDpt centerImagePoint(xSum/groundControlPoints.size(), ySum/groundControlPoints.size());
//	ossim_float64 deltaX			= 0.0;
//	ossim_float64 deltaY			= 0.0;
//	ossim_float64 maxDeltaX		= 0.0;
//	ossim_float64 maxDeltaY		= 0.0;
//	ossim_float64 deltaLat       = 0.0;
//	ossim_float64 deltaLon       = 0.0;
//	ossim_float64 deltaHeight    = 0.0;
//	ossim_float64 maxDeltaLat    = 0.0;
//	ossim_float64 maxDeltaLon    = 0.0;
//	ossim_float64 maxDeltaHeight = 0.0;
//	ossim_float64 heightTest       = 0.0;
//	for(c = 0; c < groundControlPoints.size(); ++c)
//	{
//		deltaX = (imagePoints[c].x - centerImagePoint.x);
//		deltaY = (imagePoints[c].y - centerImagePoint.y);
//		deltaLat = (groundControlPoints[c].latd()-centerGround.latd());
//		deltaLon = (groundControlPoints[c].lond()-centerGround.lond());
//		deltaHeight = (groundControlPoints[c].hgt-centerGround.hgt);
//		x[c] = deltaLon;
//		y[c] = deltaLat;
//		z[c] = deltaHeight;
//		if(fabs(deltaX) > maxDeltaX) maxDeltaX          = fabs(deltaX);
//		if(fabs(deltaY) > maxDeltaY) maxDeltaY          = fabs(deltaY);
//		if(fabs(deltaLat) > maxDeltaLat) maxDeltaLat          = fabs(deltaLat);
//		if(fabs(deltaLon) > maxDeltaLon) maxDeltaLon          = fabs(deltaLon);
//		if(fabs(deltaHeight) > maxDeltaHeight) maxDeltaHeight  = fabs(deltaHeight);
//	}
//	bool elevationEnabled = theUseElevationFlag;
//	if(fabs(maxDeltaHeight) < FLT_EPSILON) elevationEnabled = false;
//	if(maxDeltaHeight < 1.0) maxDeltaHeight = 1.0;
//	if(!elevationEnabled)
//	{
//		maxDeltaHeight = 1.0/DBL_EPSILON;
//		centerGround.height(0.0);
//	}
//	for(c = 0; c < groundControlPoints.size(); ++c)
//	{
//		x[c] /= maxDeltaLon;
//		y[c] /= maxDeltaLat;
//		z[c] /= maxDeltaHeight;
//		f[0][c] = (imagePoints[c].x - centerImagePoint.x)/maxDeltaX;
//		f[1][c] = (imagePoints[c].y - centerImagePoint.y)/maxDeltaY;
//	}
//	theLatScale    = maxDeltaLat;
//	theLonScale    = maxDeltaLon;
//	theHeightScale = maxDeltaHeight;
//	theImageOffset = centerImagePoint;
//	theImageScale  = ossimDpt(maxDeltaX,
//		maxDeltaY);
//	theGroundOffset = centerGround;
//	if(ossim::isnan(theGroundOffset.height()))
//	{
//		theGroundOffset.height(0.0);
//	}
//	std::vector<double> coeffx;
//	std::vector<double> coeffy;
//	NEWMAT::ColumnVector coeffxVec;
//	NEWMAT::ColumnVector coeffyVec;
//	double thumb = 0.5;
//	solveCoefficients(coeffxVec,
//		f[0],
//		x,
//		y,
//		z,
//		thumb / theImageScale.x);
//
//	solveCoefficients(coeffyVec,
//		f[1],
//		x,
//		y,
//		z,
//		thumb / theImageScale.y);
//	coeffx.resize(coeffxVec.Nrows());
//	coeffy.resize(coeffyVec.Nrows());
//
//	for(c = 0; c < coeffx.size();++c)
//	{
//		coeffx[c] = coeffxVec[c];
//		coeffy[c] = coeffyVec[c];
//	}
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
//	ossim_float64  sumSquareError = 0.0;
//	ossim_uint32 idx = 0;
//	for (idx = 0; idx<imagePoints.size(); idx++)
//	{
//		ossim_float64 imageX = ((eval(theXNumCoeffs, x[idx], y[idx], z[idx])/
//			eval(theXDenCoeffs, x[idx], y[idx], z[idx]))*theImageScale.x) + theImageOffset.x;
//
//		ossim_float64 imageY = ((eval(theYNumCoeffs, x[idx], y[idx], z[idx])/
//			eval(theYDenCoeffs, x[idx], y[idx], z[idx]))*theImageScale.y) + theImageOffset.y;
//
//		ossimDpt evalPt(imageX, imageY);
//		ossim_float64 len = (evalPt - imagePoints[idx]).length();
//
//		sumSquareError += (len*len);
//	}
//	theError = sqrt(sumSquareError/imagePoints.size());
//}

void ossimRpcSolver::solveCoefficients(const std::vector<ossimDpt>& imagePoints,
									   const std::vector<ossimGpt>& groundControlPoints,
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

	// compute the image bounds for the given image points
	//
	ossimDrect rect(imagePoints);

	// get the widtha dn height that will be used
	// in data normalization
	//
	ossim_float64 w = rect.width();
	ossim_float64 h = rect.height();

	// setup scales for normalization
	//   ossim_float64 xScale = w/2.0;
	//   ossim_float64 yScale = h/2.0;

	// get the shift for the cneter of the data
	ossimDpt centerImagePoint  = rect.midPoint();

	double latSum=0.0;
	double lonSum=0.0;
	double heightSum=0.0;

	// find the center ground  Use elevation only if its enabled
	//
	for(c = 0; c < groundControlPoints.size();++c)
	{
		if(ossim::isnan(groundControlPoints[c].latd()) == false)
		{
			latSum += groundControlPoints[c].latd();
		}
		if(ossim::isnan(groundControlPoints[c].lond()) == false)
		{
			lonSum += groundControlPoints[c].lond();
		}
		if(!groundControlPoints[c].isHgtNan())
		{
			if(theUseElevationFlag)
			{
				heightSum += groundControlPoints[c].height();
			}
		}
	}

	// set the center ground for the offset
	//
	ossimGpt centerGround(latSum/groundControlPoints.size(),
		lonSum/groundControlPoints.size(),
		heightSum/groundControlPoints.size());

	// set up ground scales and deltas for normalization
	//
	//   ossim_float64 latScale       = 0.0;
	//   ossim_float64 lonScale       = 0.0;
	//   ossim_float64 heightScale    = 0.0;
	ossim_float64 deltaLat       = 0.0;
	ossim_float64 deltaLon       = 0.0;
	ossim_float64 deltaHeight    = 0.0;
	ossim_float64 maxDeltaLat    = 0.0;
	ossim_float64 maxDeltaLon    = 0.0;
	ossim_float64 maxDeltaHeight = 0.0;
	ossim_float64 heightTest       = 0.0;
	for(c = 0; c < groundControlPoints.size(); ++c)
	{
		deltaLat = (groundControlPoints[c].latd()-centerGround.latd());
		deltaLon = (groundControlPoints[c].lond()-centerGround.lond());
		if(!groundControlPoints[c].isHgtNan())
		{
			if(theUseElevationFlag)
			{
				deltaHeight = groundControlPoints[c].height() - centerGround.height();
				heightTest  = groundControlPoints[c].height();
			}
			else
			{
				deltaHeight = 0.0;
				heightTest  = 0.0;
			}
		}
		else
		{
			deltaHeight = 0.0;
		}
		f[0][c] = (imagePoints[c].x - centerImagePoint.x)/(w/2.0);
		f[1][c] = (imagePoints[c].y - centerImagePoint.y)/(h/2.0);

		x[c] = deltaLon;
		y[c] = deltaLat;
		z[c] = deltaHeight;

		if(fabs(deltaLat) > maxDeltaLat) maxDeltaLat          = fabs(deltaLat);
		if(fabs(deltaLon) > maxDeltaLon) maxDeltaLon          = fabs(deltaLon);
		if(fabs(heightTest) > maxDeltaHeight) maxDeltaHeight  = fabs(heightTest);
	}
	bool elevationEnabled = theUseElevationFlag;
	// always normalize, except if too small
	//
	// if max delta is less than a degree set it to 1 degree.
	//
	if(maxDeltaLat < 1.0)    maxDeltaLat = 1.0;
	// if max delta is less than 1 degree then set it to 1.0 degree
	if(maxDeltaLon < 1.0)    maxDeltaLon = 1.0;

	if(fabs(maxDeltaHeight) < FLT_EPSILON) elevationEnabled = false;
	// if max delta is less than a meter then set it to a meter.
	if(maxDeltaHeight < 1.0) maxDeltaHeight = 1.0;

	// set the height scale to something pretty large
	if(!elevationEnabled)
	{
		maxDeltaHeight = 1.0/DBL_EPSILON;
		centerGround.height(0.0);
	}
	// normalize the ground points
	for(c = 0; c < groundControlPoints.size(); ++c)
	{
		x[c] /= maxDeltaLon;
		y[c] /= maxDeltaLat;
		z[c] /= maxDeltaHeight;
	}

	theLatScale    = maxDeltaLat;
	theLonScale    = maxDeltaLon;
	theHeightScale = maxDeltaHeight;

	theImageOffset = centerImagePoint;
	theImageScale  = ossimDpt(w/2.0,
		h/2.0);
	theGroundOffset = centerGround;

	if(ossim::isnan(theGroundOffset.height()))
	{
		theGroundOffset.height(0.0);
	}

	// now lets solve the coefficients
	//
	std::vector<double> coeffx;
	std::vector<double> coeffy;

	NEWMAT::ColumnVector coeffxVec;
	NEWMAT::ColumnVector coeffyVec;
	// perform a least squares fit for sample values found in f
	// given the world values with variables x, y, z
	//
	solveCoefficients(coeffxVec,
		f[0],
		x,
		y,
		z);


	// perform a least squares fit for line values found in f
	// given the world values with variables x, y, z
	//
	solveCoefficients(coeffyVec,
		f[1],
		x,
		y,
		z);

	coeffx.resize(coeffxVec.Nrows());
	coeffy.resize(coeffyVec.Nrows());

	for(c = 0; c < coeffx.size();++c)
	{
		coeffx[c] = coeffxVec[c];
		coeffy[c] = coeffyVec[c];
	}
	// there are 20 numerator coefficients
	// and 19 denominator coefficients
	// I believe that the very first one for the
	// denominator coefficients is fixed at 1.0
	//
	std::copy(coeffx.begin(),
		coeffx.begin()+20,
		theXNumCoeffs.begin());
	std::copy(coeffx.begin()+20,
		coeffx.begin()+39,
		theXDenCoeffs.begin()+1);
	std::copy(coeffy.begin(),
		coeffy.begin()+20,
		theYNumCoeffs.begin());
	std::copy(coeffy.begin()+20,
		coeffy.begin()+39,
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
		ossim_float64 x = (groundControlPoints[idx].lond() - theGroundOffset.lond())/theLonScale;
		ossim_float64 y = (groundControlPoints[idx].latd() - theGroundOffset.latd())/theLatScale;
		ossim_float64 z = (groundControlPoints[idx].height() - theGroundOffset.height())/theHeightScale;

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

ossimImageGeometry* ossimRpcSolver::createRpcModel()const
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
ossimImageGeometry* ossimRpcSolver::createRpcXyz2RcModel(ossimMapProjection* proj)const
{
	ossimRpcXyz2RcModel* model = new ossimRpcXyz2RcModel;
	model->m_proj = proj;

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
ossimImageGeometry* ossimRpcSolver::createRpcRcz2XyModel(ossimMapProjection* proj)const
{
	ossimRpcRcz2XyModel* model = new ossimRpcRcz2XyModel;
	model->m_proj = proj;

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
ossimImageGeometry* ossimRpcSolver::createRpcProjection()const
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
const std::vector<double>& ossimRpcSolver::getImageXNumCoefficients()const
{
   return theXNumCoeffs;
}
const std::vector<double>& ossimRpcSolver::getImageXDenCoefficients()const
{
   return theXDenCoeffs;
}
const std::vector<double>& ossimRpcSolver::getImageYNumCoefficients()const
{
   return theYNumCoeffs;
}
const std::vector<double>& ossimRpcSolver::getImageYDenCoefficients()const
{
   return theYDenCoeffs;
}
double ossimRpcSolver::getImageXOffset()const
{
   return theImageOffset.x;
}
double ossimRpcSolver::getImageYOffset()const
{
   return theImageOffset.y;
}
double ossimRpcSolver::getLatOffset()const
{
   return theGroundOffset.latd();
}
double ossimRpcSolver::getLonOffset()const
{
   return theGroundOffset.lond();
}
double ossimRpcSolver::getHeightOffset()const
{
   return theGroundOffset.height();
}
double ossimRpcSolver::getImageXScale()const
{
   return theImageScale.x;
}
double ossimRpcSolver::getImageYScale()const
{
   return theImageScale.y;
}
double ossimRpcSolver::getLatScale()const
{
   return theLatScale;
}
double ossimRpcSolver::getLonScale()const
{
   return theLonScale;
}
double ossimRpcSolver::getHeightScale()const
{
   return theHeightScale;
}
double ossimRpcSolver::getRmsError()const
{
   return theError;
}
void ossimRpcSolver::solveInitialCoefficients(NEWMAT::ColumnVector& coeff,
                                              const std::vector<double>& f,
                                              const std::vector<double>& x,
                                              const std::vector<double>& y,
                                              const std::vector<double>& z)const
{
   ossim_uint32 idx = 0;
   NEWMAT::Matrix m;
   NEWMAT::ColumnVector r((int)f.size());
   for(idx = 0; idx < f.size(); ++idx)
   {
      r[idx] = f[idx];
   }
   setupSystemOfEquations(m,
                          r,
                          x,
                          y,
                          z);
   
   coeff = invert(m.t()*m)*m.t()*r;
}

//loong
//void ossimRpcSolver::solveCoefficients(NEWMAT::ColumnVector& coeff,
//									  const std::vector<double>& f,
//									  const std::vector<double>& x,
//									  const std::vector<double>& y,
//									  const std::vector<double>& z,
//									  double thumb)const
//{
//	ossim_uint32 idx = 0;
//	NEWMAT::Matrix m;
//	NEWMAT::ColumnVector r(f.size());
//	for(idx = 0; idx < f.size(); ++idx)
//	{
//		r[idx] = f[idx];
//	}
//
//	setupSystemOfEquations(m,
//		r,
//		x,
//		y,
//		z);
//
//	vector<int> removedColsTotal;
//	NEWMAT::ColumnVector xSimplify = NestedEstimate(m, r, removedColsTotal, thumb);
//	coeff.ReSize(39);
//	coeff = 0.0;
//	int nlen = xSimplify.Nrows();
//	for(int i = 0;i < nlen;i++)
//	{
//		coeff[removedColsTotal[i]] = xSimplify[i];
//	}
//}


using namespace arma;
using namespace std;
using namespace mlpack;
using namespace mlpack::regression;
void ossimRpcSolver::solveCoefficients(NEWMAT::ColumnVector& coeff,
									   const std::vector<double>& f,
									   const std::vector<double>& x,
									   const std::vector<double>& y,
									   const std::vector<double>& z)const
{
	// this is an iterative  linear least square fit.  We really pobably need
	// a nonlinear fit instead
	//
	ossim_uint32 idx = 0;
	NEWMAT::Matrix m;
	NEWMAT::ColumnVector r(f.size());
	for(idx = 0; idx < f.size(); ++idx)
	{
		r[idx] = f[idx];
	}

	setupSystemOfEquations(m,
		r,
		x,
		y,
		z);
	int nCols = m.Ncols();
	int nRows = m.Nrows();
	arma::mat matX(nRows, nCols);
	arma::vec matY(nRows);

	for (int i = 0;i < nRows;++i)
	{
		matY[i] = r(i+1);
		for (int j = 0;j < nCols;++j)
		{
			matX(i,j) = m(i+1,j+1);
		}
	}

	// Make sure y is oriented the right way.
	if (matY.n_rows == 1)
		matY = trans(matY);
	if (matY.n_cols > 1)
		Log::Fatal << "Only one column or row allowed in responses file!" << endl;

	if (matY.n_elem != matX.n_rows)
		Log::Fatal << "Number of responses must be equal to number of rows of X!"
		<< endl;

	double lambda1 = 1e-6;
	double lambda2 = 0;
	// Do LARS.
	LARS lars(false, lambda1, lambda2);
	vec beta;
	lars.Regress(matX, matY.unsafe_col(0), beta, false /* do not transpose */);

	coeff.ReSize(beta.n_elem);
	for(int i = 0;i < beta.n_elem;i++)
	{
		coeff[i] = beta[i];
	}

	//coeff = invert(m.t()*m)*m.t()*r;
}

NEWMAT::Matrix ossimRpcSolver::invert(const NEWMAT::Matrix& m)const
{
   ossim_uint32 idx = 0;
   NEWMAT::DiagonalMatrix d;
   NEWMAT::Matrix u;
   NEWMAT::Matrix v;
   NEWMAT::SVD(m, d, u, v, true, true);
   
   for(idx=0; idx < (ossim_uint32)d.Ncols(); ++idx)
   {
      if(d[idx] > FLT_EPSILON)
      {
         d[idx] = 1.0/d[idx];
      }
      else
      {
         d[idx] = 0.0;
      }
   }
   return v*d*u.t();
}
void ossimRpcSolver::setupSystemOfEquations(NEWMAT::Matrix& equations,
                                            const NEWMAT::ColumnVector& f,
                                            const std::vector<double>& x,
                                            const std::vector<double>& y,
                                            const std::vector<double>& z)const
{
   ossim_uint32 idx;
   equations.ReSize(f.Nrows(),
                    39);
   
   for(idx = 0; idx < (ossim_uint32)f.Nrows();++idx)
   {
      equations[idx][0]  = 1;
      equations[idx][1]  = x[idx];
      equations[idx][2]  = y[idx];
      equations[idx][3]  = z[idx];
      equations[idx][4]  = x[idx]*y[idx];
      equations[idx][5]  = x[idx]*z[idx];
      equations[idx][6]  = y[idx]*z[idx];
      equations[idx][7]  = x[idx]*x[idx];
      equations[idx][8]  = y[idx]*y[idx];
      equations[idx][9]  = z[idx]*z[idx];
      equations[idx][10] = x[idx]*y[idx]*z[idx];
      equations[idx][11] = x[idx]*x[idx]*x[idx];
      equations[idx][12] = x[idx]*y[idx]*y[idx];
      equations[idx][13] = x[idx]*z[idx]*z[idx];
      equations[idx][14] = x[idx]*x[idx]*y[idx];
      equations[idx][15] = y[idx]*y[idx]*y[idx];
      equations[idx][16] = y[idx]*z[idx]*z[idx];
      equations[idx][17] = x[idx]*x[idx]*z[idx];
      equations[idx][18] = y[idx]*y[idx]*z[idx];
      equations[idx][19] = z[idx]*z[idx]*z[idx];
      equations[idx][20] = -f[idx]*x[idx];
      equations[idx][21] = -f[idx]*y[idx];
      equations[idx][22] = -f[idx]*z[idx];
      equations[idx][23] = -f[idx]*x[idx]*y[idx];
      equations[idx][24] = -f[idx]*x[idx]*z[idx];
      equations[idx][25] = -f[idx]*y[idx]*z[idx];
      equations[idx][26] = -f[idx]*x[idx]*x[idx];
      equations[idx][27] = -f[idx]*y[idx]*y[idx];
      equations[idx][28] = -f[idx]*z[idx]*z[idx];
      equations[idx][29] = -f[idx]*x[idx]*y[idx]*z[idx];
      equations[idx][30] = -f[idx]*x[idx]*x[idx]*x[idx];
      equations[idx][31] = -f[idx]*x[idx]*y[idx]*y[idx];
      equations[idx][32] = -f[idx]*x[idx]*z[idx]*z[idx];
      equations[idx][33] = -f[idx]*x[idx]*x[idx]*y[idx];
      equations[idx][34] = -f[idx]*y[idx]*y[idx]*y[idx];
      equations[idx][35] = -f[idx]*y[idx]*z[idx]*z[idx];
	  equations[idx][36] = -f[idx]*x[idx]*x[idx]*z[idx];	// loong 重大错误
	  equations[idx][37] = -f[idx]*y[idx]*y[idx]*z[idx];
      equations[idx][38] = -f[idx]*z[idx]*z[idx]*z[idx];
   }
}
void ossimRpcSolver::setupWeightMatrix(NEWMAT::DiagonalMatrix& result, // holds the resulting weights
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
	   row[17] = x[idx]*x[idx]*z[idx];	// loong 重大错误
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
double ossimRpcSolver::eval(const std::vector<double>& coeff,
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
ossimRefPtr<ossimNitfRegisteredTag> ossimRpcSolver::getNitfRpcBTag() const
{
   ossimNitfRpcBTag* rpcbTag = new ossimNitfRpcBTag();
   rpcbTag->setSuccess(true);
   rpcbTag->setErrorBias(0.0);
   rpcbTag->setErrorRand(0.0);
   rpcbTag->setLineOffset(static_cast<ossim_uint32>(getImageYOffset()));
   rpcbTag->setSampleOffset(static_cast<ossim_uint32>(getImageXOffset()));
   rpcbTag->setGeodeticLatOffset(getLatOffset());
   rpcbTag->setGeodeticLonOffset(getLonOffset());
   rpcbTag->setGeodeticHeightOffset(
      static_cast<ossim_int32>(getHeightOffset()));
   rpcbTag->setLineScale(static_cast<ossim_uint32>(getImageYScale()));
   rpcbTag->setSampleScale(static_cast<ossim_uint32>(getImageXScale()));
   rpcbTag->setGeodeticLatScale(getLatScale());
   rpcbTag->setGeodeticLonScale(getLonScale());
   rpcbTag->setGeodeticHeightScale(static_cast<ossim_int32>(getHeightScale()));
   rpcbTag->setLineNumeratorCoeff(getImageYNumCoefficients());
   
   rpcbTag->setLineDenominatorCoeff(getImageYDenCoefficients());
   rpcbTag->setSampleNumeratorCoeff(getImageXNumCoefficients());
   rpcbTag->setSampleDenominatorCoeff(getImageXDenCoefficients());
   ossimRefPtr<ossimNitfRegisteredTag> tag = rpcbTag;
   
   return tag;
}

//loong
NEWMAT::Matrix ossimRpcSolver::WithoutCol(const NEWMAT::Matrix& X,  int n)const
{
	int nrows = X.Nrows();
	int ncols = X.Ncols();
	NEWMAT::Matrix result(nrows, ncols - 1);
	if(n >= ncols)
	{
		return result;
	}
	for(int i = 0;i < nrows;i++)
	{
		for(int j = 0;j < ncols;j++)
		{
			if(j < n)
			{
				result[i][j] = X[i][j];
			}
			else if(j > n)
			{
				result[i][j-1] = X[i][j];
			}
		}
	}
	return result;
}

NEWMAT::ColumnVector ossimRpcSolver::WithoutCol(const NEWMAT::ColumnVector& y,  int n)const
{
	int nrows = y.Nrows();
	NEWMAT::ColumnVector result(nrows - 1);
	if(n >= nrows)
	{
		return result;
	}
	for(int i = 0;i < nrows;i++)
	{
		if(i < n)
		{
			result[i] = y[i];
		}
		else if(i > n)
		{
			result[i - 1] = y[i];
		}
	}
	return result;
}


NEWMAT::ColumnVector ossimRpcSolver::setRpcCoeffs(const NEWMAT::ColumnVector& y, vector<int> removedCols, int nCoeff)const
{
	NEWMAT::ColumnVector coeff(nCoeff);
	int nlen = static_cast<int>(removedCols.size());
	if(y.Nrows()+nlen != nCoeff) return coeff;

	std::vector<int> indexList;
	for(int i = 0;i < nCoeff;i++) indexList.push_back(i);
	std::vector<int> removedColsOrigin;
	for(int i = 0;i < nlen;i++)
	{
		removedColsOrigin.push_back(indexList[removedCols[i]]);
		indexList.erase(indexList.begin()+removedCols[i]);
	}
	sort(removedColsOrigin.begin(), removedColsOrigin.end());

	int posRemoved = 0;
	int posNotRemoved = 0;
	for(int i = 0;i < nCoeff;i++)
	{
		if(posRemoved < nlen && i == removedColsOrigin[posRemoved])
		{
			coeff[i] = 0.0;
			posRemoved++;
		}
		else
		{
			coeff[i] = y[posNotRemoved];
			posNotRemoved++;
		}
	}
	return coeff;
}

double ossimRpcSolver::findMax(const std::vector<double> x, int& maxPos)const
{
	double xmax = 0.0;
	for(int i = 0;i < static_cast<int>(x.size());i++)
	{
		if(xmax < x[i])
		{
			xmax = x[i];
			maxPos = i;
		}
	}
	return xmax;
}

double ossimRpcSolver::ColumnVectorMean(const NEWMAT::ColumnVector& y)const
{
	int n = y.Nrows();
	double sum = 0.0;
	for(int i = 0;i < n;i++)
	{
		sum += y[i];
	}
	return sum/n;
}


bool ossimRpcSolver::SetColumnOfMat(NEWMAT::Matrix& X, int pos, const NEWMAT::ColumnVector& y)const
{
	int n = y.Nrows();
	if(X.Nrows() != n || pos >= X.Ncols()) return false;

	for(int i = 0;i < n;i++)
	{
		X[i][pos] = y[i];
	}

	return true;
}

bool ossimRpcSolver::AppendColumn(NEWMAT::Matrix& X, const NEWMAT::ColumnVector& y)const
{
	int nRows = X.Nrows();
	int nCols = X.Ncols();
	if(y.Nrows() != nRows) return false;
	NEWMAT::Matrix tmp = X;
	X.CleanUp();
	X.ReSize(nRows, nCols+1);
	for(int i = 0;i < nRows;i++)
	{
		for(int j = 0;j < nCols;j++)
		{
			X[i][j] = tmp[i][j];
		}
		X[i][nCols] = y[i];
	}
	return true;
}

double ossimRpcSolver::LogicSign(const NEWMAT::ColumnVector& y, const NEWMAT::ColumnVector& x)const
{
	NEWMAT::ColumnVector b = invert(x.t()*x)*x.t()*y;

	double y_bar = ColumnVectorMean(y);
	// ssr
	NEWMAT::ColumnVector f = x*b;
	NEWMAT::ColumnVector ff = f-y_bar;
	NEWMAT::Matrix ssr = ff.t() * ff;

	//sst
	NEWMAT::ColumnVector yy = y-y_bar;
	NEWMAT::Matrix sst = yy.t() * yy;

	double R2 = ssr[0][0] / sst[0][0];
	double logic_sign = 1000*R2/(1-R2);
	return logic_sign;
}

vector<double> ossimRpcSolver::LogicSigns(const NEWMAT::ColumnVector& y, const NEWMAT::Matrix& X)const
{
	int nCols = X.Ncols();
	vector<double> logic_signList(nCols);
	for(int i = 0;i < nCols;i++)
	{
		logic_signList[i] = LogicSign(y, X.Column(i+1));
	}
	return logic_signList;
}

NEWMAT::ColumnVector ossimRpcSolver::NestedEstimate(const NEWMAT::Matrix& X, const NEWMAT::ColumnVector& y, vector<int>& removedColsTotal, double thumb)const
{
	int nCols = X.Ncols();
	int nRows = X.Nrows();
	vector<int> posTable(nCols, 1);
	vector<int> totalIndex(nCols, -1);
	NEWMAT::ColumnVector epsilon = y;
	NEWMAT::Matrix B = X;
	int absPos;
	TotalIndex(posTable, 0, absPos);
	totalIndex[0] = 0;
	NEWMAT::Matrix NewMat = B.Column(1);
	B = WithoutCol(B, 0);
	vector<int> colRecord;
	colRecord.push_back(0);
	vector<double> logic_signList = LogicSigns(epsilon, B);
	int maxPos;
	double maxValue = findMax(logic_signList, maxPos);

	int nIter = 1;
	double rms = 0.0;
	while(nIter < nRows-1 && nIter < nCols)
	{
		TotalIndex(posTable, maxPos, absPos);
		totalIndex[nIter] = absPos;
		colRecord.push_back(maxPos);
		AppendColumn(NewMat, B.Column(maxPos+1));
		B = WithoutCol(B, maxPos);

		NEWMAT::ColumnVector coeff = invert(NewMat.t()*NewMat)*NewMat.t()*y;
		////debug
		//fstream fs;
		//fs.open("NewMat.txt", ios_base::out);
		//fs<<NewMat<<endl;
		//fs.close();
		//cout<<coeff<<endl;
		NEWMAT::ColumnVector y_hat = SampleLine(X, coeff, totalIndex);
		//epsilon = epsilon - NewMat*invert(NewMat.t()*NewMat)*NewMat.t()*epsilon;
		epsilon = y - y_hat;

		double rmsNew = sqrt(epsilon.SumSquare()/nRows);
		if(fabs(rms - rmsNew) < thumb && rmsNew < thumb) break;
		rms = rmsNew;

		logic_signList = LogicSigns(epsilon, B);
		maxValue = findMax(logic_signList, maxPos);
		nIter++;
	}

	int nlen = static_cast<int>(colRecord.size());
	vector<int> indexList;
	int nCoeff = 39;
	for(int i = 0;i < nCoeff;i++) indexList.push_back(i);

	removedColsTotal.clear();
	for(int i = 0;i < nlen;i++)
	{
		removedColsTotal.push_back(indexList[colRecord[i]]);
		indexList.erase(indexList.begin()+colRecord[i]);
	}
	sort(removedColsTotal.begin(), removedColsTotal.end());
	NEWMAT::Matrix A(nRows, nlen);
	for(int i = 0;i < nlen;i++)
	{
		SetColumnOfMat(A, i, X.Column(removedColsTotal[i]+1));
	}
	NEWMAT::ColumnVector xSimplify = invert(A.t()*A)*A.t()*y;
	fstream fs;
	fs.open("debug.txt", ios_base::out);
	fs<<"A:\n"<<A<<endl;
	fs<<"xSimplify:\n"<<xSimplify<<endl;
	fs.close();
	return xSimplify;
}

void ossimRpcSolver::TotalIndex(vector<int>& posTable, int relPos, int& absPos)const
{
	int n = posTable.size();
	int ncount = 0;
	for(int i = 0;i < n;i++)
	{
		if(posTable[i] == 1)
		{
			if(ncount == relPos)
			{
				posTable[i] = 0;
				absPos = i;
				return;
			}
			ncount++;
		}
	}
	absPos = -1;
}

NEWMAT::ColumnVector ossimRpcSolver::SampleLine(const NEWMAT::Matrix& X, const NEWMAT::ColumnVector& coeff, const vector<int>& totalIndex)const
{
	int nCols = coeff.Nrows();
	int nRows = X.Nrows();
	NEWMAT::ColumnVector a(20);
	NEWMAT::ColumnVector b(20);
	a = 0.0;
	b = 0.0;
	b[0] = 1.0;

	for(int i = 0;i < nCols;i++)
	{
		int col = totalIndex[i];
		int newCol = col - 20;
		if(col < 20)
		{
			a[col] = coeff[i];
		}
		else
		{
			b[col - 19] = coeff[i];
		}
	}
	NEWMAT::Matrix b1 = X.Columns(1, 20);
	NEWMAT::ColumnVector r(nRows);
	for(int i = 0;i < nRows;i++)
	{
		NEWMAT::Matrix x1 = b1.Row(i+1)*a;
		NEWMAT::Matrix x2 = b1.Row(i+1)*b;
		r[i] = x1[0][0] / x2[0][0];
	}
	////debug
	//fstream fs;
	//fs.open("SampleLine.txt", ios_base::out);
	//fs<<"b1:\n"<<b1<<endl;
	//fs<<"a:\n"<<a<<endl;
	//fs<<"b:\n"<<b<<endl;
	//fs<<"r:\n"<<r<<endl;
	//fs.close();
	return r;
}

//void ossimRpcSolver::solveCoefficients_Rcz2Xy(const std::vector<ossimDpt>& imagePoints,
//											 const std::vector<ossimGpt>& groundControlPoints,
//											 const ossimDpt& imageShift)
//{
//	if((imagePoints.size() != groundControlPoints.size()))
//	{
//		return;
//	}
//	std::vector<double> f[2];
//	std::vector<double> line;
//	std::vector<double> samp;
//	std::vector<double> z;
//	ossim_uint32 c = 0;
//	f[0].resize(imagePoints.size());
//	f[1].resize(imagePoints.size());
//	line.resize(imagePoints.size());
//	samp.resize(imagePoints.size());
//	z.resize(imagePoints.size());
//	ossimDrect rect(imagePoints);
//
//	double xSum = 0.0;
//	double ySum = 0.0;
//	double latSum=0.0;
//	double lonSum=0.0;
//	double heightSum=0.0;
//	for(c = 0; c < groundControlPoints.size();++c)
//	{
//		if(ossim::isnan(imagePoints[c].x) == false)
//		{
//			xSum += imagePoints[c].x;
//		}
//		if(ossim::isnan(imagePoints[c].y) == false)
//		{
//			ySum += imagePoints[c].y;
//		}
//		if(ossim::isnan(groundControlPoints[c].latd()) == false)
//		{
//			latSum += groundControlPoints[c].latd();
//		}
//		if(ossim::isnan(groundControlPoints[c].lond()) == false)
//		{
//			lonSum += groundControlPoints[c].lond();
//		}
//		if(!groundControlPoints[c].isHgtNan())
//		{
//			if(theUseElevationFlag)
//			{
//				heightSum += groundControlPoints[c].height();
//			}
//		}
//	}
//	ossimGpt centerGround(latSum/groundControlPoints.size(),
//		lonSum/groundControlPoints.size(),
//		heightSum/groundControlPoints.size());
//	ossimDpt centerImagePoint(xSum/groundControlPoints.size(), ySum/groundControlPoints.size());
//	ossim_float64 deltaX			= 0.0;
//	ossim_float64 deltaY			= 0.0;
//	ossim_float64 maxDeltaX		= 0.0;
//	ossim_float64 maxDeltaY		= 0.0;
//	ossim_float64 deltaLat       = 0.0;
//	ossim_float64 deltaLon       = 0.0;
//	ossim_float64 deltaHeight    = 0.0;
//	ossim_float64 maxDeltaLat    = 0.0;
//	ossim_float64 maxDeltaLon    = 0.0;
//	ossim_float64 maxDeltaHeight = 0.0;
//	ossim_float64 heightTest       = 0.0;
//	for(c = 0; c < groundControlPoints.size(); ++c)
//	{
//		deltaX = (imagePoints[c].x - centerImagePoint.x);
//		deltaY = (imagePoints[c].y - centerImagePoint.y);
//		deltaLat = (groundControlPoints[c].latd()-centerGround.latd());
//		deltaLon = (groundControlPoints[c].lond()-centerGround.lond());
//		deltaHeight = (groundControlPoints[c].hgt-centerGround.hgt);
//
//		if(fabs(deltaX) > maxDeltaX) maxDeltaX          = fabs(deltaX);
//		if(fabs(deltaY) > maxDeltaY) maxDeltaY          = fabs(deltaY);
//		if(fabs(deltaLat) > maxDeltaLat) maxDeltaLat          = fabs(deltaLat);
//		if(fabs(deltaLon) > maxDeltaLon) maxDeltaLon          = fabs(deltaLon);
//		if(fabs(deltaHeight) > maxDeltaHeight) maxDeltaHeight  = fabs(deltaHeight);
//	}
//	bool elevationEnabled = theUseElevationFlag;
//	if(fabs(maxDeltaHeight) < FLT_EPSILON) elevationEnabled = false;
//	if(maxDeltaHeight < 1.0) maxDeltaHeight = 1.0;
//	if(!elevationEnabled)
//	{
//		maxDeltaHeight = 1.0/DBL_EPSILON;
//		centerGround.height(0.0);
//	}
//	for(c = 0; c < groundControlPoints.size(); ++c)
//	{
//		line[c] = (imagePoints[c].x - centerImagePoint.x)/maxDeltaX;
//		samp[c] = (imagePoints[c].y - centerImagePoint.y)/maxDeltaY;
//		z[c] = (groundControlPoints[c].hgt-centerGround.hgt)/maxDeltaHeight;
//
//		f[0][c] = (groundControlPoints[c].latd()-centerGround.latd())/maxDeltaLat;
//		f[1][c] = (groundControlPoints[c].lond()-centerGround.lond())/maxDeltaLon;
//	}
//	theLatScale    = maxDeltaLat;
//	theLonScale    = maxDeltaLon;
//	theHeightScale = maxDeltaHeight;
//	theImageOffset = centerImagePoint;
//	theImageScale  = ossimDpt(maxDeltaX,
//		maxDeltaY);
//	theGroundOffset = centerGround;
//	if(ossim::isnan(theGroundOffset.height()))
//	{
//		theGroundOffset.height(0.0);
//	}
//	std::vector<double> coeffx;
//	std::vector<double> coeffy;
//	NEWMAT::ColumnVector coeffxVec;
//	NEWMAT::ColumnVector coeffyVec;
//	double thumb = 0.5;	//pixel
//	solveCoefficients(coeffxVec,
//		f[0],
//		line,
//		samp,
//		z,
//		thumb / theImageScale.x);
//
//	solveCoefficients(coeffyVec,
//		f[1],
//		line,
//		samp,
//		z,
//		thumb / theImageScale.y);
//	coeffx.resize(coeffxVec.Nrows());
//	coeffy.resize(coeffyVec.Nrows());
//
//	for(c = 0; c < coeffx.size();++c)
//	{
//		coeffx[c] = coeffxVec[c];
//		coeffy[c] = coeffyVec[c];
//	}
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
//	ossim_float64  sumSquareError = 0.0;
//	ossim_uint32 idx = 0;
//	for (idx = 0; idx<imagePoints.size(); idx++)
//	{
//		ossim_float64 X = ((eval(theXNumCoeffs, line[idx], samp[idx], z[idx])/
//			eval(theXDenCoeffs, line[idx], samp[idx], z[idx]))*theLatScale) + centerGround.lat;
//
//		ossim_float64 Y = ((eval(theYNumCoeffs, line[idx], samp[idx], z[idx])/
//			eval(theYDenCoeffs, line[idx], samp[idx], z[idx]))*theLonScale) + centerGround.lon;
//
//		ossimDpt evalPt(X, Y);
//		ossimDpt p1(groundControlPoints[idx].lat, groundControlPoints[idx].lon);
//		ossim_float64 len = (evalPt - p1).length();
//
//		sumSquareError += (len*len);
//	}
//	theError = sqrt(sumSquareError/imagePoints.size());
//}