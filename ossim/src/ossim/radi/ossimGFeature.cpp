#include <iostream>
#include <sstream>
#include <ossim/base/ossimGpt.h>
#include <ossim/base/ossimEcefPoint.h>
#include <ossim/base/ossimEcefVector.h>
#include <ossim/base/ossimDms.h>
#include <ossim/base/ossimDatum.h>
#include <ossim/base/ossimGeoidManager.h>
#include <ossim/base/ossimEllipsoid.h>
#include <ossim/base/ossimGeocent.h>
#include <ossim/radi/ossimGFeature.h>
#pragma warning(push)
#pragma warning(disable : 4482)


ossimGFeature::ossimGFeature(const ossimGPoint& Gpt)
{
	m_featureType = ossimFeatureType::ossimPointType;
	m_Points.clear();
	strId = Gpt.strId;
	m_Points.push_back(Gpt.point);
}

ossimGFeature::ossimGFeature(const ossimGLine& Line)
{
	m_featureType = ossimFeatureType::ossimStraightLineType;
	m_Points.clear();
	m_Points.push_back(Line.first);
	m_Points.push_back(Line.second);
}

ossimGFeature::ossimGFeature(const ossimGArea& Area)
{
	m_featureType = ossimFeatureType::ossimPolygonType;
	m_Points = Area.m_Points;
}

ossimGFeature::ossimGFeature(const ossimGFeature& src)
: strId(src.strId),
m_Points(src.m_Points),
m_featureType(src.m_featureType)
{
}
const ossimGFeature& ossimGFeature::operator = (const ossimGFeature &aFeature)
{
	strId = aFeature.strId;
	m_featureType = aFeature.m_featureType;
	m_Points = aFeature.m_Points;
	return *this;
}
#pragma warning(pop)