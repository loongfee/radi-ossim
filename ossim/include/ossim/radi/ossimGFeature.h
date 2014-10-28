#ifndef ossimGFeature_HEADER
#define ossimGFeature_HEADER
#include <iosfwd>
#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/base/ossimDatumFactory.h>
#include <ossim/base/ossimString.h>
#include <ossim/radi/ossimDFeature.h>
#pragma warning(push)
#pragma warning(disable : 4482)
class OSSIMDLLEXPORT ossimDatum;
class OSSIMDLLEXPORT ossimEcefPoint;

class OSSIMDLLEXPORT ossimGpt;
// Area calss
class OSSIMDLLEXPORT ossimGArea
{
public:
	vector<ossimGpt> m_Points;
	ossimString strId;
public:
	/*!
	* Constructors
	*/
	ossimGArea(){strId = "NULL";};
	ossimGArea(vector<ossimGpt> Points, ossimString astrId = "NULL"){m_Points = Points; strId = astrId;};
	bool inArea(ossimGpt gpt);
};

class OSSIMDLLEXPORT ossimGLine
{
public:
	ossimGpt first;
	ossimGpt second;
	ossimString strId;
public:
	/*!
	* Constructors
	*/
	ossimGLine(){strId = "NULL";};
	ossimGLine(ossimGpt Gpt1, ossimGpt Gpt2, ossimString astrId = "NULL"){first = Gpt1;second = Gpt2; strId = astrId;};
};

class OSSIMDLLEXPORT ossimGPoint
{
public:
	ossimGpt point;
	ossimString strId;
public:
	/*!
	* Constructors
	*/
	ossimGPoint(){point.makeNan(); strId = "NULL";};
	ossimGPoint(const ossimGpt& gpt, ossimString astrId = "NULL"){point = gpt; strId = astrId;};
};

class OSSIM_DLL ossimGFeature
{
public:

	ossimFeatureType m_featureType;
	vector<ossimGpt> m_Points;
	ossimString strId;
	ossimGFeature(){m_featureType = ossimUnknown;};
	ossimGFeature(ossimFeatureType type){m_featureType = type;};
	ossimGFeature(const ossimGPoint& Gpt);		//initialize as Gpt
	ossimGFeature(const ossimGLine& Line);	//initialize as Line
	ossimGFeature(const ossimGArea& Area);	//initialize as Area
	ossimGFeature(const ossimGFeature& src);
	const ossimGFeature& operator = (const ossimGFeature &aFeature);
	void makeNan(){m_featureType = ossimUnknown;m_Points.clear();}
	bool isNan()const
	{
	   return (m_Points.size() == 0);
	}
};
#pragma warning(pop)
#endif
