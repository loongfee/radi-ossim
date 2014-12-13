#ifndef ossimDFeature_HEADER
#define ossimDFeature_HEADER
#include <iosfwd>
#include <string>
#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimCommon.h>
#include <ossim/base/ossimString.h>
#pragma warning(push)
#pragma warning(disable : 4482)
class OSSIMDLLEXPORT ossimIpt;
class OSSIMDLLEXPORT ossimFpt;
class OSSIMDLLEXPORT ossimDpt3d;
class OSSIMDLLEXPORT ossimGpt;
class OSSIMDLLEXPORT ossimDpt;

bool isPointOnSegment(ossimDpt pt, ossimDpt a, ossimDpt b);
ossimDpt Distance2Segment(ossimDpt pt, ossimDpt a, ossimDpt b, ossimDpt* intersection = NULL);
double CrossMultiplication(ossimDpt pt, ossimDpt a, ossimDpt b);

#ifndef PI
#define PI 3.14159265
#endif

enum ossimFeatureType
{
	ossimPointType,
	ossimStraightLineType,
	ossimFreeLineType,
	ossimPolygonType,
	ossimUnknown
};

// Line class
class OSSIMDLLEXPORT ossimDLine
{
public:
	ossimDpt getFirstPoint()const{return first;};
	ossimDpt getSecondPoint()const{return second;};
	ossimString getId()const{return strId;};
	double getRho()const{return rho;};
	double getTheta()const{return theta;};

	void setPoints(ossimDpt Dpt1, ossimDpt Dpt2);
public:
	/*!
	* Constructors
	*/
	ossimDLine(){};
	ossimDLine(ossimDpt Dpt1, ossimDpt Dpt2);
	// 点到直线的距离
	double DistanceFromPoint(ossimDpt pt)	const;
	double DistanceFromPoint2(const ossimDpt& p) const;
	ossimDpt DistanceFromSegment(const ossimDLine& l) const;
	ossimDpt DistanceAndSegment(const ossimDLine& l) const;
	void toPoint(ossimDpt *pt) const;
	void getPointDistanceDeriv(ossimDpt dpt, double* partial_x, double* partial_y, double hdelta =1e-6) const;
	void getSegmentDistanceDeriv(const ossimDLine& l, double* partial_x, double* partial_y, double hdelta =1e-6) const;
	void getDistanceDeriv(ossimDpt* partial_x, ossimDpt* partial_y) const;
protected:	
	ossimDpt first;
	ossimDpt second;
	ossimString strId;
	
/************************************************************************/
/* 
	rho = -x * sin(theta) + y * cos(theta)
*/
/************************************************************************/
	double rho;
	double theta;
	friend class ossimDFeature;
};

// Area class
class OSSIMDLLEXPORT ossimDArea
{
public:
	vector<ossimDpt> m_Points;
	ossimString strId;
public:
	/*!
	* Constructors
	*/
	ossimDArea(){};
	ossimDArea(const vector<ossimDpt>& Points);
	double DistanceFromPoint(ossimDpt pt)	const;
	// 面到面的距离, index为area中到面的距离最大的顶点序号
	void DistanceFromArea(const ossimDArea &area, double* dist) const;
	void DistanceFromArea(const ossimDArea &area, ossimDpt* dist) const;
	int X_Intersection(ossimDpt pt, ossimDpt a, ossimDpt b, ossimDpt* intersection = NULL)const;
private:
	double Distance2Straightline(ossimDpt pt, ossimDpt a, ossimDpt b, ossimDpt* intersection = NULL)const;
};

class OSSIMDLLEXPORT ossimDPoint
{
public:
	ossimDpt point;
	ossimString strId;
public:
	/*!
	* Constructors
	*/
	ossimDPoint(){point.makeNan(); strId = "NULL";};
	ossimDPoint(const ossimDpt& gpt, ossimString astrId = "NULL"){point = gpt; strId = astrId;};
};

class OSSIM_DLL ossimDFeature
{
public:
	
	ossimFeatureType m_featureType;
	vector<ossimDpt> m_Points;
	ossimString strId;
   /**
    * Constructor.  The values are assumed to be in DEGREES.
    */
   ossimDFeature();
   ossimDFeature(const ossimFeatureType& type);
   ossimDFeature(const ossimDPoint& Dpt);		//initialize as Gpt
   ossimDFeature(const ossimDLine& Line);	//initialize as Line
   ossimDFeature(const ossimDArea& Area);	//initialize as Area
   ossimDFeature(const ossimDFeature& src);
   const ossimDFeature& operator = (const ossimDFeature &aFeature);
   void makeNan(){m_featureType = ossimUnknown;m_Points.clear();}
   bool isNan()const
   {
	   return (m_Points.size() == 0);
   }
};
#pragma warning(pop)
#endif
