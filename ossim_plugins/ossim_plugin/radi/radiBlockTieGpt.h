#ifndef radiBlockTieGpt_HEADER
#define radiBlockTieGpt_HEADER

#include <ossim/base/ossimString.h>
#include <ossim/base/ossimTieGpt.h>
#include <ossim/base/ossimTieGptSet.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/projection/ossimSensorModel.h>
#include <ossimPluginConstants.h>


namespace ossimplugins
{
/************************************************************************/
/*                          Block Adjustment                            */
/*	Author: loong
	Date: 2010.01.20
	Modified on 2014.08.21
*/
/************************************************************************/

/***************************************************************************/
/*
 *                              DEFINES
*/
typedef std::pair<ossimDpt, ossimDpt> ossimDline;
typedef std::pair<ossimGpt, ossimGpt> ossimGline;

class OSSIM_PLUGINS_DLL radiBlockTieGpt : public ossimTieGpt
{
public:
	enum point_type
	{
		unknown_tie_image_points = 0,
		known_ground_control_points,
		known_ground_check_points,
	};
	int m_SlaveId;
	int m_MasterId;

	ossimString m_ID;
	point_type m_nType;						// 0: unknown corresponding image points;
											// 1: known ground control points;
											// 2: known corresponding image points;
	//vector< int > m_ImageIndices;		// begin with 0
	//vector< ossimDpt > m_DptList;		// corresponding with m_ImageIndices
	vector< pair<int, ossimDpt> > m_DptList;
	int m_nUnknownIndex;				// -1:	unknown corresponding image points;
										// >-1:	known ground control points or known
										// corresponding image points (begin with 0);
public:
	/*!
	* Constructors
	*/
	//radiBlockTieGpt();
	//radiBlockTieGpt(ossimTieGpt tieGpt);

	radiBlockTieGpt()
	:ossimTieGpt(),
	m_SlaveId(1),
	m_MasterId(2)
	{
	
	};
	radiBlockTieGpt(ossimTieGpt tieGpt)
		:ossimTieGpt(tieGpt),
		m_SlaveId(1),
		m_MasterId(2)
	{
		//*this = dynamic_cast(tieGpt);
	};

	void setSlaveId(const int& id){m_SlaveId = id;};
	int getSlaveId()const{return m_SlaveId;};
	void setMasterId(const int& id){m_MasterId = id;};
	int getMasterId()const{return m_MasterId;};
	void setPointType(const point_type& ptType){m_nType = ptType;};
	point_type getPointType()const{return m_nType;};

   /**
    * GML feauture (XML) serialization
    */
   ossimRefPtr<ossimXmlNode> exportAsGmlNode(ossimString aGmlVersion="2.1.2")const;
   bool importFromGmlNode(ossimRefPtr<ossimXmlNode> aGmlNode, ossimString aGmlVersion="2.1.2");
};
} // end of namespace ossimplugins

#endif /* radiBlockTieGpt_HEADER */