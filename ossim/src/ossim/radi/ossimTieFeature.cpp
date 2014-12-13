#include <iostream>
#include <iomanip>
#include <ossim/base/ossimTieGpt.h>
#include <ossim/base/ossimString.h>
#include <ossim/base/ossimDatum.h>
#include <ossim/base/ossimNotifyContext.h>
#include <ossim/radi/ossimTieFeature.h>
#pragma warning(push)
#pragma warning(disable : 4482)

inline ossimTieFeature::ossimTieFeature(const ossimGFeature& gptFeature, const ossimDFeature& dptFeature, const ossim_float64& aScore, const ossimString& astrId/*=""*/)
: ossimReferenced(),
score(aScore),
strId(astrId)
{
	m_groundFeature = gptFeature;
	m_imageFeature = dptFeature;
	updateTieFeatureType();
}

inline ossimTieFeature::ossimTieFeature(const ossimTieFeature& tieFeature) :
ossimReferenced(tieFeature),
score(tieFeature.score),
strId(tieFeature.strId)
{
	if (this != &tieFeature)
	{
		ossimReferenced::operator=(tieFeature);
		m_groundFeature = tieFeature.m_groundFeature;
		m_imageFeature = tieFeature.m_imageFeature;
		updateTieFeatureType();
	}
}

inline void ossimTieFeature::setGroundFeature(ossimGFeature gptFeature)
{
	m_groundFeature = gptFeature;
	updateTieFeatureType();
}
inline const ossimGFeature& ossimTieFeature::getGroundFeature()const
{
	return m_groundFeature;
}
inline ossimGFeature& ossimTieFeature::refGroundFeature()
{
	return m_groundFeature;
}
inline void ossimTieFeature::setImageFeature(ossimDFeature dptFeature)
{
	m_imageFeature = dptFeature;
	updateTieFeatureType();
}
inline const ossimDFeature& ossimTieFeature::getImageFeature()const
{
	return m_imageFeature;
}
inline ossimDFeature& ossimTieFeature::refImageFeature()
{
	return m_imageFeature;
}

void ossimTieFeature::updateTieFeatureType()
{
	if(ossimFeatureType::ossimPointType == m_imageFeature.m_featureType 
		&& ossimFeatureType::ossimPointType == m_groundFeature.m_featureType)
	{
		m_tieFeatureType = ossimTiePointPoint;
	}
	else if(ossimFeatureType::ossimPointType == m_imageFeature.m_featureType 
		&& ossimFeatureType::ossimStraightLineType == m_groundFeature.m_featureType)
	{
		m_tieFeatureType = ossimTiePointLine;
	}
	else if(ossimFeatureType::ossimPointType == m_imageFeature.m_featureType 
		&& ossimFeatureType::ossimPolygonType == m_groundFeature.m_featureType)
	{
		m_tieFeatureType = ossimTiePointArea;
	}
	else if(ossimFeatureType::ossimStraightLineType == m_imageFeature.m_featureType 
		&& ossimFeatureType::ossimPointType == m_groundFeature.m_featureType)
	{
		m_tieFeatureType = ossimTieLinePoint;
	}
	else if(ossimFeatureType::ossimStraightLineType == m_imageFeature.m_featureType 
		&& ossimFeatureType::ossimStraightLineType == m_groundFeature.m_featureType)
	{
		m_tieFeatureType = ossimTieLineLine;
	}
	else if(ossimFeatureType::ossimStraightLineType == m_imageFeature.m_featureType 
		&& ossimFeatureType::ossimPolygonType == m_groundFeature.m_featureType)
	{
		m_tieFeatureType = ossimTieLineArea;
	}
	else if(ossimFeatureType::ossimPolygonType == m_imageFeature.m_featureType 
		&& ossimFeatureType::ossimPointType == m_groundFeature.m_featureType)
	{
		m_tieFeatureType = ossimTieAreaPoint;
	}
	else if(ossimFeatureType::ossimPolygonType == m_imageFeature.m_featureType 
		&& ossimFeatureType::ossimStraightLineType == m_groundFeature.m_featureType)
	{
		m_tieFeatureType = ossimTieAreaLine;
	}
	else if(ossimFeatureType::ossimPolygonType == m_imageFeature.m_featureType 
		&& ossimFeatureType::ossimPolygonType == m_groundFeature.m_featureType)
	{
		m_tieFeatureType = ossimTieAreaArea;
	}
	else
	{
		m_tieFeatureType = ossimTieAreaArea;
	}
}
#pragma warning(pop)