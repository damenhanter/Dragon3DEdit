#pragma once
#include "datatype.h"
#include "Vector2.h"

//////////////////////////////////////////////////////////////////////////
//2D������ֱ��
//////////////////////////////////////////////////////////////////////////

class CParamLine2D
{
public:
	CParamLine2D(void);
	~CParamLine2D(void);

	void InitParamLine2D(const CPoint2D& pa,const CPoint2D& pb);
	inline CPoint2D ComputeParamLine2D(float t)
	{
		return CPoint2D(p0.x+p1.x*t,p0.y+p1.y*t);
	}
	int IntersectParamLine2D(const CParamLine2D& L,float& t1,float& t2);
	int IntersectParamLine2D(const CParamLine2D& L,CPoint2D& pt);
public:
	CPoint2D p0;       //������ֱ�ߵ����
	CPoint2D p1;       //������ֱ�ߵ��յ�
	CVector2 v;        //�߶εķ�������
};

