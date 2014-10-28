#ifndef ossimTieFeature_HEADER
#define ossimTieFeature_HEADER
#include <iostream>
#include <ossim/base/ossimXmlNode.h>
#include <ossim/base/ossimGpt.h>
#include <ossim/base/ossimReferenced.h>
#include <ossim/radi/ossimGFeature.h>
#include <ossim/radi/ossimDFeature.h>
#pragma warning(push)
#pragma warning(disable : 4482)
class OSSIMDLLEXPORT ossimDpt;
class OSSIMDLLEXPORT ossimString;
/**
 * storage class for tie point between ground and image
 * based on ossimGpt
 * + GML feature (OGC) serialization
 *
 * NOTES
 * accuracy is not stored here (need to derive object if need a per-point accuracy) 
 * GML storage is WGS84 only, it stores above ellipsoid height (m)
 *
 * TODO : 
 * -support other datum (easy) / ground projection (big change in OSSIM)
 * -unify with ossimTDpt
 */
class OSSIMDLLEXPORT ossimTieFeature :  ossimReferenced
{
public:
	enum ossimTieFeatureType
	{
		ossimTiePointPoint,
		ossimTieLinePoint,
		ossimTieAreaPoint,
		ossimTiePointLine,
		ossimTieLineLine,
		ossimTieAreaLine,
		ossimTiePointArea,
		ossimTieLineArea,
		ossimTieAreaArea,
		ossimTieUnknown
	};
	ossimGFeature m_groundFeature;
	ossimDFeature m_imageFeature;
	ossimString strId;
	ossim_float64 score;
   inline ossimTieFeature() : 
      ossimReferenced(),
		  score(0.0),
	  strId("")
      {}
   inline ossimTieFeature(const ossimGFeature& gptFeature, const ossimDFeature& dptFeature, const ossim_float64& aScore, const ossimString& astrId="NULL"); 
         
   inline ossimTieFeature(const ossimTieFeature& tieFeature); 
   inline ~ossimTieFeature() {}
	inline const ossimTieFeature& operator=(const ossimTieFeature&);
   inline void					setGroundFeature(ossimGFeature gptFeature);
   inline const ossimGFeature& getGroundFeature()const;
   inline		ossimGFeature& refGroundFeature();
   inline void					setImageFeature(ossimDFeature dptFeature);
   inline const ossimDFeature& getImageFeature()const;
   inline		ossimDFeature& refImageFeature();
   inline void            setId(const ossimString& s) { strId=s; }
   inline void            setScore(const ossim_float64& s) { score=s; }
   inline const ossim_float64& getScore()const             { return score; }
   inline const ossimString& getId()const             { return strId; }
   inline       ossim_float64& refScore()                  { return score; }
   void makeNan() 
      {
       m_groundFeature.makeNan();
       m_imageFeature.makeNan();
       score=ossim::nan();
      }
   
   bool hasNans()const
   {
      return (m_groundFeature.isNan() || m_imageFeature.isNan() || (ossim::isnan(score)));
   }
   
   bool isNan()const
   {
      return (m_groundFeature.isNan() && m_imageFeature.isNan() && (ossim::isnan(score)));
   }
   ossimTieFeatureType getTieFeatureType() const{return m_tieFeatureType;};
   void updateTieFeatureType();
protected:
   ossimTieFeatureType m_tieFeatureType;
};
inline const ossimTieFeature& ossimTieFeature::operator=(const ossimTieFeature& feature)
{
   if (this != &feature)
   {
      ossimReferenced::operator=(feature);
	  m_groundFeature = feature.m_groundFeature;
	  m_imageFeature = feature.m_imageFeature;
      score = feature.score;
	  strId = feature.strId;
	  updateTieFeatureType();
   }
   return *this;
}
#pragma warning(pop)
#endif /* #ifndef ossimTieFeature_HEADER */
