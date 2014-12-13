#include <radi/radiBlockTieGpt.h>

namespace ossimplugins
{
	//constants for GML 2.1.2
	const char* MASTER_GML2      = "Master";
	const char* SLAVE_GML2       = "Slave";
	const char* POINT_GML2       = "gml:Point";
	const char* SCORE_GML2       = "Score";
	const char* COORD_GML2       = "gml:Coord";
	const char* COORDINATES_GML2 = "gml:Coordinates";
	const char* TIEPOINTS_GML2  = "SimpleTiePoint";
	const char* ID_GML2  = "Id";
	const char* TYPE_GML2 = "Type";
	const char* TIE = "Tie";
	const char* CONTROL = "Control";
	const char* CHECK = "Check";
	const char* UNKNOWN = "Unknown";

	ossimRefPtr<ossimXmlNode>
		radiBlockTieGpt::exportAsGmlNode(ossimString aGmlVersion)const
	{   
		ossimRefPtr<ossimXmlNode> node(new ossimXmlNode);
		// check datum to be WGS84
		if ( !(datum()->operator==(*(ossimDatumFactory::instance()->wgs84()))) )
		{
			ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::exportAsGmlNode datum must be WGS84\n";
			return node;
		}
		// check nans in lon/lat and in tie
		if (isLatNan() || isLonNan() || tie.hasNans())
		{
			ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::exportAsGmlNode positions have nan\n";
			return node;
		}
		if (aGmlVersion[static_cast<std::string::size_type>(0)] == '2')
		{
			node->setTag(TIEPOINTS_GML2);
			// check the point type
			// note: no "fid" attribute (feature id")
			//store Ground Point WGS84 + height above ellipsoid in meters
			if (point_type::unknown_tie_image_points == m_nType)
			{
				node->addNode(TYPE_GML2,ossimString(TIE));
				for (std::vector< pair<int, ossimDpt> >::const_iterator iter = m_DptList.begin();
					iter != m_DptList.end();++iter)
				{
					// store image tie point
					ossimRefPtr<ossimXmlNode> tieNode =  node->addChildNode(ossimString(TIE));
					ossimRefPtr<ossimXmlNode> tcoord =  tieNode->addNode(ossimString(POINT_GML2) + "/" + COORD_GML2);
					tcoord->addChildNode("X",ossimString::toString(iter->second.x));
					tcoord->addChildNode("Y",ossimString::toString(iter->second.y));
					tieNode->addAttribute(ID_GML2, ossimString::toString(iter->first));
				}
			}
			else if(point_type::known_ground_control_points == m_nType
				||point_type::known_ground_check_points == m_nType)
			{
				if (point_type::known_ground_control_points == m_nType)
				{
					node->addNode(TYPE_GML2,ossimString(CONTROL));
				}
				else
				{
					node->addNode(TYPE_GML2,ossimString(CHECK));
				}
				ossimRefPtr<ossimXmlNode> gcoord =  node->addNode(ossimString(MASTER_GML2) + "/" + POINT_GML2 + "/" + COORD_GML2 );
				gcoord->addChildNode("X",ossimString::toString(lond()));
				gcoord->addChildNode("Y",ossimString::toString(latd()));
				if (!isHgtNan())
				{
					gcoord->addChildNode("Z",ossimString::toString(height())); //above ellipsoid
				}
				node->findFirstNode(MASTER_GML2)->addAttribute(ID_GML2, ossimString::toString(m_MasterId));
				// store image tie point
				ossimRefPtr<ossimXmlNode> tcoord =  node->addNode(ossimString(SLAVE_GML2) + "/" + POINT_GML2 + "/" + COORD_GML2);
				tcoord->addChildNode("X",ossimString::toString(tie.x));
				tcoord->addChildNode("Y",ossimString::toString(tie.y));
				node->findFirstNode(SLAVE_GML2)->addAttribute(ID_GML2, ossimString::toString(m_SlaveId));
			}
			else
			{
				node->addNode(TYPE_GML2,ossimString(UNKNOWN));
				ossimRefPtr<ossimXmlNode> gcoord =  node->addNode(ossimString(MASTER_GML2) + "/" + POINT_GML2 + "/" + COORD_GML2 );
				gcoord->addChildNode("X",ossimString::toString(lond()));
				gcoord->addChildNode("Y",ossimString::toString(latd()));
				if (!isHgtNan())
				{
					gcoord->addChildNode("Z",ossimString::toString(height())); //above ellipsoid
				}
				node->findFirstNode(MASTER_GML2)->addAttribute(ID_GML2, ossimString::toString(m_MasterId));
				// store image tie point
				ossimRefPtr<ossimXmlNode> tcoord =  node->addNode(ossimString(SLAVE_GML2) + "/" + POINT_GML2 + "/" + COORD_GML2);
				tcoord->addChildNode("X",ossimString::toString(tie.x));
				tcoord->addChildNode("Y",ossimString::toString(tie.y));
				node->findFirstNode(SLAVE_GML2)->addAttribute(ID_GML2, ossimString::toString(m_SlaveId));
			}

			//store score (change name to confidence?)
			node->addNode(SCORE_GML2,ossimString::toString(score));
		} else {
			ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::exportAsGmlNode Unsupported GML version : " << aGmlVersion <<"\n";
		}
		return node;
	}

	bool
		radiBlockTieGpt::importFromGmlNode(ossimRefPtr<ossimXmlNode> aGmlNode, ossimString aGmlVersion)
	{
		//assuming datum is EPSG:4326 (aka WGS84)
		//feature has to be a SimpleTiePoint feature
		//TBD : add support for coord instead of coordinates
		//TBD : more robust type checks (for X,Y,Z and score) - create extra protected function

		//clear data
		makeNan();
		tie.makeNan();
		score = 0;

		if (aGmlVersion[static_cast<std::string::size_type>(0)] == '2')
		{
			//read point type
			ossimRefPtr<ossimXmlNode> ptType = aGmlNode->findFirstNode(TYPE_GML2);
			if (!ptType.valid() || 0 == strcmp(ptType->getText(), TIE))
			{
				vector< ossimRefPtr< ossimXmlNode > > tieNodes;
				aGmlNode->findChildNodes(TIE, tieNodes);
				for (int i = 0;i < (int)tieNodes.size();++i)
				{
					int imageId = tieNodes[i]->getAttributeValue(ID_GML2).toInt();
					ossimRefPtr<ossimXmlNode> in = tieNodes[i]->findFirstNode(ossimString(POINT_GML2));
					ossimRefPtr<ossimXmlNode> icoord = in->findFirstNode(COORD_GML2);
					ossimDpt dpt;
					if (icoord.valid())
					{
						//read coord
						ossimRefPtr<ossimXmlNode> ix = icoord->findFirstNode("X");
						if (ix.valid())
						{
							dpt.x = ossimString(ix->getText()).toDouble();
						} else {
							ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode no image X found in coord\n";
							return false;
						}
						ossimRefPtr<ossimXmlNode> iy = icoord->findFirstNode("Y");
						if (iy.valid())
						{
							dpt.y = ossimString(iy->getText()).toDouble();
						} else {
							ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode no image Y found in coord\n";
							return false;
						}
						//don't read Z value (shouldn't be any)
					}
					else {
						//try to read coordinates
						//TBD
						ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode gml:coordinates not developped yet for image\n";
						return false;
					}
					m_DptList.push_back(pair<int, ossimDpt>(imageId, dpt));
				}
				m_nType = unknown_tie_image_points;
			}
			else
			{
				//read ground point
				m_MasterId = aGmlNode->findFirstNode(ossimString(MASTER_GML2))->getAttributeValue(ID_GML2).toInt();
				ossimRefPtr<ossimXmlNode> gn = aGmlNode->findFirstNode(ossimString(MASTER_GML2)+"/"+POINT_GML2);
				ossimRefPtr<ossimXmlNode> gcoord = gn->findFirstNode(COORD_GML2);
				if (gcoord.valid())
				{
					//read coord
					ossimRefPtr<ossimXmlNode> gx = gcoord->findFirstNode("X");
					if (gx.valid())
					{
						lond(ossimString(gx->getText()).toDouble());
					} else {
						ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode no ground X found in coord\n";
						return false;
					}
					ossimRefPtr<ossimXmlNode> gy = gcoord->findFirstNode("Y");
					if (gy.valid())
					{
						latd(ossimString(gy->getText()).toDouble());
					} else {
						ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode no ground Y found in coord\n";
						return false;
					}
					ossimRefPtr<ossimXmlNode> gz = gcoord->findFirstNode("Z");
					if (gz.valid())
					{
						height(ossimString(gz->getText()).toDouble());
					} // no Z value is possible
				}
				else {
					//try to read coordinates
					//TBD
					ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode gml:coordinates not developped yet for ground\n";
					return false;
				}

				//read image point
				m_SlaveId = aGmlNode->findFirstNode(ossimString(SLAVE_GML2))->getAttributeValue(ID_GML2).toInt();
				ossimRefPtr<ossimXmlNode> in = aGmlNode->findFirstNode(ossimString(SLAVE_GML2)+"/"+POINT_GML2);
				ossimRefPtr<ossimXmlNode> icoord = in->findFirstNode(COORD_GML2);
				if (icoord.valid())
				{
					//read coord
					ossimRefPtr<ossimXmlNode> ix = icoord->findFirstNode("X");
					if (ix.valid())
					{
						tie.x = ossimString(ix->getText()).toDouble();
					} else {
						ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode no image X found in coord\n";
						return false;
					}
					ossimRefPtr<ossimXmlNode> iy = icoord->findFirstNode("Y");
					if (iy.valid())
					{
						tie.y = ossimString(iy->getText()).toDouble();
					} else {
						ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode no image Y found in coord\n";
						return false;
					}
					//don't read Z value (shouldn't be any)
				}
				else {
					//try to read coordinates
					//TBD
					ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode gml:coordinates not developped yet for image\n";
					return false;
				}
				if (0 == strcmp(ptType->getText(), CONTROL))
				{
					m_nType = known_ground_control_points;
				}
				else
				{
					m_nType = known_ground_check_points;
				}
			}


			//read score
			ossimRefPtr<ossimXmlNode> scoren = aGmlNode->findFirstNode(SCORE_GML2);
			if (scoren.valid())
			{
				score = ossimString(scoren->getText()).toDouble();
			} else {
				score = 0.0;
			}
			return true;
		} else {
			ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGpt::importFromGmlNode Unsupported GML version : " << aGmlVersion <<"\n";
			return false;
		}
	}
} // end of namespace ossimplugins