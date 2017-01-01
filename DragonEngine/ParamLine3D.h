#pragma once
#include "datatype.h"
#include "Vector3.h"

//////////////////////////////////////////////////////////////////////////
//3D������ֱ��
//////////////////////////////////////////////////////////////////////////

class CParamLine3D
{
public:
	CParamLine3D(void);
	~CParamLine3D(void);

	void InitParamLine3D(const CPoint3D& p0,const CPoint3D& p1);
	CPoint3D ComputeParamLine3D(float t);
public:
	CPoint3D p0;   //������ֱ�ߵ����
	CPoint3D p1;   //������ֱ�ߵ��յ�
	CVector3 v;    //�߶εķ�������
};
