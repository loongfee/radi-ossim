#include <iostream>
#include <iomanip>
#include <sstream>
#include <ossim/base/ossimDpt.h>
#include <ossim/base/ossimDpt3d.h>
#include <ossim/base/ossimIpt.h>
#include <ossim/base/ossimFpt.h>
#include <ossim/radi/ossimDFeature.h>
#pragma warning(push)
#pragma warning(disable : 4482)



//////////////////////////////////////////////////////////////////////////
////////////////////////////ossimDLine////////////////////////////////

ossimDLine::ossimDLine(ossimDpt Dpt1, ossimDpt Dpt2)
{
	setPoints(Dpt1, Dpt2);
}

void ossimDLine::setPoints(ossimDpt Dpt1, ossimDpt Dpt2)
{
	first = Dpt1;
	second = Dpt2;

	rho = 0;
	//角度为对应直线与X轴的夹角
	theta = 0;
	if( first.x == second.x)
	{
		// 如果与Y轴平行
		rho = (double)first.x;
		theta = PI / 2.0;
	} 
	else
	{
		theta = atan((second.y - first.y) / (double)(second.x - first.x));
		//float t0 = (line[1].x * line[0].y - line[0].x * line[1].y) * cos(theta);
		//float t1 = -line[0].x * sin(theta) + line[0].y * cos(theta);
		//float t2 = -line[1].x * sin(theta) + line[1].y * cos(theta);
		rho = -first.x * sin(theta) + first.y * cos(theta);
	}
}

double ossimDLine::DistanceFromPoint(ossimDpt pt) const
{
	ossimDpt disVector;
	ossimDpt intersection_Tmp;
	double len0_2 = sqrt(ossim::square<double>(first.x - second.x) + ossim::square<double>(first.y - second.y));
	double len0 = sqrt(len0_2);
	if(len0 < DBL_EPSILON)
	{
		// 如果两点相同
		return sqrt(ossim::square<double>(first.x - pt.x) + ossim::square<double>(first.y - pt.y));
	}
	//面积法求距离
	return fabs(CrossMultiplication(pt, first, second) / len0);
}

double ossimDLine::DistanceFromPoint2(const ossimDpt& p) const
{
	double dist = -p.x * sin(theta) + p.y * cos(theta) - rho;
	return dist;
	//double dist = -p.x * sin(theta) + p.y * cos(theta) - rho;
	//double dx = fabs(dist*cos(theta));
	//double dy = fabs(dist*sin(theta));
	//return ossimDpt(dx, dy);
}

ossimDpt ossimDLine::DistanceFromSegment(const ossimDLine& l) const
{
	double  d1 = DistanceFromPoint2(l.first);
	double d2 = DistanceFromPoint2(l.second);
	return ossimDpt(d1, d2);
	//ossimDpt d1 = DistanceFromPoint2(l.first);
	//ossimDpt d2 = DistanceFromPoint2(l.second);
	//return ossimDpt(sqrt(d1.x*d1.x+d1.y*d1.y), sqrt(d2.x*d2.x+d2.y*d2.y));
	//return ossimDpt(0.5*(d1.x+d2.x), 0.5*(d1.y+d2.y));
}

ossimDpt ossimDLine::DistanceAndSegment(const ossimDLine& l) const
{
	//ossimDpt base_point1 = ossimDpt(0.0, 0.0);
	//ossimDpt base_point2 = ossimDpt(6000.0, 6000.0);
	//double dist1 =DistanceFromPoint2(base_point1) - l.DistanceFromPoint2(base_point1);
	//double dist2 =DistanceFromPoint2(base_point2) - l.DistanceFromPoint2(base_point2);
	//return ossimDpt(dist1, dist2);
	////return DistanceFromSegment(l);
	////ossimDpt dpt1 = DistanceFromSegment(l);
	////ossimDpt dpt2 = l.DistanceFromSegment(*this);
	////double d1 = sqrt(dpt1.x*dpt1.x + dpt1.y*dpt1.y);
	////double d2 = sqrt(dpt2.x*dpt2.x + dpt2.y*dpt2.y);
	////return ossimDpt(d1, d2);

	ossimDpt d1 = DistanceFromSegment(l);
	ossimDpt d2 = l.DistanceFromSegment(*this);
	//return ossimDpt(sqrt(d1.x*d1.x+d1.y*d1.y), sqrt(d2.x*d2.x+d2.y*d2.y));
	return ossimDpt(0.5*(fabs(d1.x)+fabs(d2.x)), 0.5*(fabs(d1.y)+fabs(d2.y)));
}

void ossimDLine::toPoint(ossimDpt *pt) const
{
	double delta_x = getFirstPoint().x - getSecondPoint().x;
	double delta_y = getFirstPoint().y - getSecondPoint().y;
	// x1*y2 - x2*y1
	double x1y2_x2y1 = getFirstPoint().x * getSecondPoint().y - getSecondPoint().x * getFirstPoint().y;
	double delta_x2 = delta_x * delta_x;
	double delta_y2 = delta_y * delta_y;
	double denominator = delta_x2+delta_y2+DBL_EPSILON;
	*pt = ossimDpt(delta_y*x1y2_x2y1/denominator, -delta_x*x1y2_x2y1/denominator);
}

void ossimDLine::getPointDistanceDeriv(ossimDpt dpt, double* partial_x, double* partial_y, double hdelta/* =1e-6*/) const
{
	*partial_x = -sin(theta);
	*partial_y = cos(theta);
	return;

	double den = 0.5/hdelta;
	double dis1 = DistanceFromPoint(ossimDpt(dpt.x + hdelta, dpt.y));
	double dis2 = DistanceFromPoint(ossimDpt(dpt.x - hdelta, dpt.y));
	*partial_x = dis1 - dis2;
	dis1 = DistanceFromPoint(ossimDpt(dpt.x, dpt.y + hdelta));
	dis2 = DistanceFromPoint(ossimDpt(dpt.x, dpt.y - hdelta));
	*partial_y = dis1 - dis2;
	*partial_x = *partial_x * den;
	*partial_y = *partial_y * den;
}

void ossimDLine::getDistanceDeriv(ossimDpt* partial_x, ossimDpt* partial_y) const
{
	double delta_x = getFirstPoint().x - getSecondPoint().x;
	double delta_y = getFirstPoint().y - getSecondPoint().y;
	double delta_x2 = delta_x * delta_x;
	double delta_y2 = delta_y * delta_y;
	double delta_xy = delta_x * delta_y;
	double denominator = delta_x2+delta_y2+DBL_EPSILON;
	double ay2 = delta_y2 / denominator;
	double axy = delta_xy / denominator;
	double ax2 = delta_x2 / denominator;
	*partial_x = ossimDpt(ay2, axy);
	*partial_y = ossimDpt(axy, ax2);
}

//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
///////////////////////////////ossimDArea////////////////////////////////
//////////////////////////////////////////////////////////////////////////
ossimDArea::ossimDArea(const vector<ossimDpt>& Points)
{
	m_Points = Points;

	// 保证闭合
	if(m_Points.size() > 1)
	{
		if(m_Points[0] != m_Points.back())
		{
			m_Points.push_back(m_Points[0]);
		}
	}
}

//
//robust and smooth Hausdorff distance (RS-HD)
double ossimDArea::DistanceFromPoint(ossimDpt pt) const
{
	int nPoint = (int)m_Points.size();
	double alpha = -10.0;
	double eps = 1.0;
	double dis = 0.0;
	for(unsigned int i = 0;i < m_Points.size() - 1;i++)
	{
		double dis_pt = sqrt((m_Points[i].x-pt.x)*(m_Points[i].x-pt.x)+(m_Points[i].y-pt.y)*(m_Points[i].y-pt.y));
		dis += pow(dis_pt+eps, alpha);
	}
	dis = pow(dis/nPoint, 1.0/alpha)-eps;
	return dis;
}

// 
int ossimDArea::X_Intersection(ossimDpt pt, ossimDpt a, ossimDpt b, ossimDpt* intersection/* = NULL*/)const
{
	ossimDpt dpt1 = a - b;
	ossimDpt dpt2 = pt - a;
	if(fabs(dpt1.y) < DBL_EPSILON)
	{
		//如果水平
		intersection->makeNan();
		return 0;
	}
	else
	{
		intersection->y = pt.y;
		intersection->x = dpt1.x * dpt2.y / dpt1.y + a.x;
		double dis = intersection->x - pt.x;
		if(dis > 0) return 1;	//右相交
		else if(dis < 0) return -1;	//左相交
		else return 0;	//不相交
	}
}

double ossimDArea::Distance2Straightline(ossimDpt pt, ossimDpt a, ossimDpt b, ossimDpt* intersection/* = NULL*/)const
{
	double dis = ossim::nan();
	ossimDpt dpt1 = a - b;
	ossimDpt dpt2 = pt - a;
	if(fabs(dpt1.x) < DBL_EPSILON && fabs(dpt1.y) < DBL_EPSILON)
	{
		// 如果两点相同
		intersection->makeNan();
		dis = sqrt((pt.x - a.x) * (pt.x - a.x) + (pt.y - a.y) * (pt.y - a.y));
		return dis;
	}
	// X方向
	if(fabs(dpt1.y) < DBL_EPSILON)
	{
		//如果水平
		intersection->makeNan();
	}
	else
	{
		intersection->y = pt.y;
		intersection->x = dpt1.x * dpt2.y / dpt1.y + a.x;
		dis = intersection->x - pt.x;
	}
	if(fabs(dpt1.x) >= DBL_EPSILON)
	{
		//如果不垂直
		if(ossim::isnan(dis))
		{
			//如果水平
			dis = dpt1.y * dpt2.x / dpt1.x - dpt2.y;
		}
		else
		{
			if(dis >= 0.0)
				dis = min(fabs(dis), fabs(dpt1.y * dpt2.x / dpt1.x - dpt2.y));
			else
				dis = -min(fabs(dis), fabs(dpt1.y * dpt2.x / dpt1.x - dpt2.y));
		}
	}
	return dis;
}

double CrossMultiplication(ossimDpt pt, ossimDpt a, ossimDpt b)
{
	return (a.x-pt.x)*(b.y-pt.y)-(b.x-pt.x)*(a.y-pt.y);
}

ossimDpt Distance2Segment(ossimDpt pt, ossimDpt a, ossimDpt b, ossimDpt* intersection/* = NULL*/) 
{
	ossimDpt disVector;
	ossimDpt intersection_Tmp;
	double len0_2 = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
	double len0 = sqrt(len0_2);
	if(len0 < DBL_EPSILON)
	{
		// 如果两点相同
		if(intersection) *intersection = a;
		disVector.x = a.x - pt.x;
		disVector.y = a.y - pt.y;
		return disVector;
	}
	double d1_2 = (a.x - pt.x) * (a.x - pt.x) + (a.y - pt.y) * (a.y - pt.y);
	double d1 = sqrt(d1_2);
	double d2_2 = (b.x - pt.x) * (b.x - pt.x) + (b.y - pt.y) * (b.y - pt.y);
	double d2 = sqrt(d2_2);
	//面积法求距离
	double d0 = fabs(CrossMultiplication(pt, a, b) / len0);
	double d0_2 = d0 * d0;
	double len1_2 = d1_2 - d0_2;
	double len1 = sqrt(len1_2);
	double len2_2 = d2_2 - d0_2;
	double len2 = sqrt(len2_2);

	if (len1_2 > len0_2 || len2_2 > len0_2)
	{
		//垂足在线段外,取端点
		if (len1_2 > len2_2)
		{
			if(intersection) *intersection = b;
			disVector.x = b.x - pt.x;
			disVector.y = b.y - pt.y;
			return disVector;
		}
		else
		{
			if(intersection) *intersection = a;
			disVector.x = a.x - pt.x;
			disVector.y = a.y - pt.y;
			return disVector;
		}
	}
	//垂足在线段上
	intersection_Tmp.x = a.x + (b.x - a.x) * len1 / len0;
	intersection_Tmp.y = a.y + (b.y - a.y) * len1 / len0;
	if(intersection)
	{
		*intersection = intersection_Tmp;
	}
	disVector.x = intersection_Tmp.x - pt.x;
	disVector.y = intersection_Tmp.y - pt.y;
	return disVector;
}
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
/////////////////////////////ossimDFeature///////////////////////////////
//////////////////////////////////////////////////////////////////////////
ossimDFeature::ossimDFeature()
{
	m_featureType = ossimUnknown;
}
ossimDFeature::ossimDFeature(const ossimFeatureType& type)
{
	m_featureType = type;
}
ossimDFeature::ossimDFeature(const ossimDPoint& Dpt)
{
	m_featureType = ossimFeatureType::ossimPointType;
	m_Points.clear();
	strId = Dpt.strId;
	m_Points.push_back(Dpt.point);
}

ossimDFeature::ossimDFeature(const ossimDLine& Line)
{
	m_featureType = ossimFeatureType::ossimStraightLineType;
	m_Points.clear();
	m_Points.push_back(Line.first);
	m_Points.push_back(Line.second);
}

ossimDFeature::ossimDFeature(const ossimDArea& Area)
{
	m_featureType = ossimFeatureType::ossimPolygonType;
	m_Points = Area.m_Points;
}

ossimDFeature::ossimDFeature(const ossimDFeature& src) :
strId(src.strId),
m_Points(src.m_Points),
m_featureType(src.m_featureType)
{
}
const ossimDFeature& ossimDFeature::operator = (const ossimDFeature &aFeature)
{
	strId = aFeature.strId;
	m_featureType = aFeature.m_featureType;
	m_Points = aFeature.m_Points;
	return *this;
}
//////////////////////////////////////////////////////////////////////////


void ossimDArea::DistanceFromArea(const ossimDArea &area, double* dist) const
{
	int nPoints = (int)area.m_Points.size();
	*dist = 0;
	for (int i = 0;i < nPoints;++i)
	{
		*dist += DistanceFromPoint(area.m_Points[i]);
	}
	*dist /= nPoints;
}

void ossimDArea::DistanceFromArea(const ossimDArea &area, ossimDpt* dist) const
{
	DistanceFromArea(area, &(dist->x));
	area.DistanceFromArea(*this, &(dist->y));
}


bool isPointOnSegment(ossimDpt pt, ossimDpt a, ossimDpt b)	
{
	// 线段左闭右开
	if(pt.isNan())
	{
		// 点无效
		return false;
	}
	ossimDpt dpt1 = a - b;
	ossimDpt dpt2 = pt - a;
	if(fabs(dpt1.x * dpt2.y - dpt1.y * dpt2.x) > DBL_EPSILON)
	{
		// 不在直线上
		return false;
	}
	else
	{
		return (pt.x >= min(a.x, b.x) && pt.x < max(a.x, b.x) && pt.y >= min(a.y, b.y) && pt.y <= max(a.y, b.y));
	}
}

#pragma warning(pop)