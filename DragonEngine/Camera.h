#pragma once
#include "Vector4.h"
#include "Plane3D.h"
#include "Matrix4x4.h"

//////////////////////////////////////////////////////////////////////////
//���֧��ŷ��ģ�ͻ�UVNģ��
//////////////////////////////////////////////////////////////////////////
class CCamera
{
public:
	CCamera(void);
	
	~CCamera(void);
	void InitCamera(int camAttr,CPoint4D& camPos,CVector4& camDir,
		CPoint4D* camTarget,float nz,float fz,float fov,float vpWidth,float vpHeight);
	void CreateCAM4DV1MatrixEuler(int rotateSeq);
	void CreateCAM4DV1MatrixUVN(int mode);
public:
	int state;  //���״̬
	int attr;   //�������
	
	CPoint4D pos;      //�������������ϵ�е�λ��
	CVector4 dir;      //ŷ���Ƕ�/UVN���ģ�͵�ע������
	
	CVector4 u;        //UVNģ�͵ĳ�������
	CVector4 v;
	CVector4 n;
	CPoint4D target;   //UVNģ�͵�Ŀ���
	 
	float fov;         //ˮƽ��ֱ�������Ұ

	float viewDist;  //ˮƽ��ֱ�Ӿ�

	//3D�ý���
	float nearZ;
	float farZ;
	CPlane3D rtPlane;
	CPlane3D ltPlane;
	CPlane3D tpPlane;
	CPlane3D btPlane;

	float viewPlaneWidth;       //��ƽ��Ŀ�Ⱥ͸߶�
	float viewPlaneHeight;

	float viewPortWidth;        //��Ļ/�ӿڵĿ�Ⱥ͸߶�
	float viewPortHeight;
	float viewPortCenterX;      //�ӿ�����
	float viewPortCenterY;

	float aspectRatio;           //��߱�

	// �任����
	CMatrix4x4 mcam;
	CMatrix4x4 mper;
	CMatrix4x4 mscr;
};
