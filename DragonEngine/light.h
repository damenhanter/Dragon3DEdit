#pragma once
#include "datatype.h"
#include "Vector4.h"

class CLight
{
public:
	CLight()
	{
		state=0;
		id=0;
		attr=0;
		pf=0;
		sp=0;
		kl=kc=kq=0;
		innerSpot=0;
		outerSpot=0;
	}
	int CreateLightV1(int state, int attr, RGBAV1 Ca, RGBAV1 Cd, RGBAV1 Cs,
		CPoint4D &pos, CVector4 &dir, float kc, float kl, float kq,
		float innerSpot, float outerSpot, int pf,int sp);
public:
	int state;
	int id;
	int attr;

	RGBAV1 Ca;        //������ǿ��
	RGBAV1 Cd;        //�������ǿ��
	RGBAV1 Cs;        //�����ǿ��
	CPoint4D pos;     //��Դλ��
	CPoint4D tpos;
	CVector4 dir;     //��Դ����
	CVector4 tdir;
	float kc,kl,kq;   //˥������
	float innerSpot;   //�۹����׶��
	float outerSpot;   //�۹����׶��
	int pf;          //�۹��ָ������
	int sp;           //���淴��ָ��
};