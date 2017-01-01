#include "StdAfx.h"
#include "ParamLine2D.h"

CParamLine2D::CParamLine2D(void)
{
}

CParamLine2D::~CParamLine2D(void)
{
}

void CParamLine2D::InitParamLine2D(const CPoint2D& pa,const CPoint2D& pb)
{
	p0=pa;p1=pb;
	v.CreateVector(pa,pb);
}
//////////////////////////////////////////////////////////////////////////
//t1,t2��ʾ������L1,L2�϶�Ӧ��tֵ
//��������ֵ: 
//0 ��ʾû�н���      1 ��ʾ�߶��ڽ�����
//2 ��ʾ�ཻ����һ���ڽ����� 3 ��ʾ�����߶�λ��ͬһ��ֱ����
//////////////////////////////////////////////////////////////////////////
int CParamLine2D::IntersectParamLine2D(const CParamLine2D &L, float &t1, float &t2)
{
	float det_L1L2=(v.x*L.v.y - v.y*L.v.x);
	if (fabs(det_L1L2)<=EPSILON_E5)
	{
		return PARAM_LINE_NO_INIERSECT;
	}
	t1 = (L.v.x*(p0.y - L.p0.y) - L.v.y*(p0.x - L.p0.x))/det_L1L2;

	t2 = (v.x*(p0.y - L.p0.y) - v.y*(p0.x - L.p0.x))/det_L1L2;

	if ((t1>=0) && (t1<=1) && (t2>=0) && (t2<=1))
	{
		return PARAM_LINE_INTERSECT_IN_SEGMENT;
	}
	else
	{
		return PARAM_LINE_INTERSECT_OUT_SEGMENT;
	}
}

int CParamLine2D::IntersectParamLine2D(const CParamLine2D &L, CPoint2D &pt)
{
	float t1, t2, det_L1L2 = (v.x*L.v.y - v.y*L.v.x);

	if (fabs(det_L1L2) <= EPSILON_E5)
	{
		return(PARAM_LINE_NO_INIERSECT);
	}
	t1 = (L.v.x*(p0.y - L.p0.y) - L.v.y*(p0.x - L.p0.x))
		/det_L1L2;

	t2 = (v.x*(p0.y - L.p0.y) -v.y*(p0.x - L.p0.x))
		/det_L1L2;

	pt.x = p0.x + v.x*t1;
	pt.y = p0.y + v.y*t1;

	if ((t1>=0) && (t1<=1) && (t2>=0) && (t2<=1))
		return(PARAM_LINE_INTERSECT_IN_SEGMENT);
	else
		return(PARAM_LINE_INTERSECT_OUT_SEGMENT);
}