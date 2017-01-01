#include "StdAfx.h"
#include "Plane3D.h"

CPlane3D::CPlane3D(void)
{
}

CPlane3D::~CPlane3D(void)
{
}

void CPlane3D::InitPlane3D(const CPoint3D &pt, CVector3 v, BOOL normalize)
{
	p0=pt;
	if (!normalize)
	{
		n=v;
	}
	else
	{
		v.Normalize();
	}
}

//////////////////////////////////////////////////////////////////////////
//��������ƽ���ϡ�����ռ䡢���Ǹ���ռ� 
//////////////////////////////////////////////////////////////////////////

float CPlane3D::IsPointInPlane3D(const CPoint3D &pt)
{
	float ret=n.x*(pt.x-p0.x)+n.y*(pt.y-p0.y)+n.z*(pt.z-p0.z);
	return ret;
}

//////////////////////////////////////////////////////////////////////////
//�������������ֱ����ƽ��Ľ���
//���������paramline3D�ϣ������t��ֵλ��[0,1]��
//������ཻ��������0��������ֱ���Ϸ���1
//���㲻���߶��Ϸ���2���߶�λ��ƽ�淵��3
//////////////////////////////////////////////////////////////////////////
int CPlane3D::IntersectParamLine3DPlane3D(const CParamLine3D &paramline3D, float &t, CPoint3D &pt)
{
	float planeDotline=n.DotProduct(paramline3D.v);

	if (fabs(planeDotline)<=EPSILON_E5)
	{
		//�߶���ƽ��ƽ�У������Ƿ���ƽ����
		if (fabs(IsPointInPlane3D(paramline3D.p0))<=EPSILON_E5)
		{
			return PARAM_LINE_INTERSECT_EVERYWHERE;
		}
		else
		{
			return PARAM_LINE_NO_INIERSECT;
		}
	}

	t=-(n.x*paramline3D.p0.x + n.y*paramline3D.p0.y + 
		n.z*paramline3D.p0.z -n.x*p0.x - n.y*p0.y - n.z*p0.z) / (planeDotline);

	pt.x=paramline3D.p0.x+paramline3D.v.x*t;
	pt.y=paramline3D.p0.y+paramline3D.v.y*t;
	pt.z=paramline3D.p0.z+paramline3D.v.z*t;

	if (t>=0.0f && t<=1.0f)
	{
		return PARAM_LINE_INTERSECT_IN_SEGMENT;
	}
	else
	{
		return PARAM_LINE_INTERSECT_OUT_SEGMENT;
	}
}