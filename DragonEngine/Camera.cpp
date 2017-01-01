#include "StdAfx.h"
#include "Camera.h"
#include "datatype.h"
#include "Vector3.h"
#include "ErrorLog.h"
extern CErrorLog errorlog;

CCamera::CCamera(void)
{
}

CCamera::~CCamera(void)
{
}

void CCamera::InitCamera(int camAttr, CPoint4D &camPos, CVector4 &camDir, CPoint4D* camTarget, 
				 float nz, float fz, float fov, float vpWidth, float vpHeight)
				 
{
	attr=camAttr;
	pos=camPos;
	dir=camDir;

	u=CVector4(1,0,0);
	v=CVector4(0,1,0);
	n=CVector4(0,0,1);
	target=CVector4(0,0,0);

	if (camTarget!=NULL)
	{
		target=*camTarget;
	}
	nearZ=nz;
	farZ=fz;

	viewPortWidth=vpWidth;
	viewPortHeight=vpHeight;
	viewPortCenterX=(vpWidth-1)/2;
	viewPortCenterY=(vpHeight-1)/2;
	aspectRatio=(float)viewPortWidth/(float)viewPortHeight;
	//���任�������óɵ�λ����
	mcam.Identity();
	mper.Identity();
	mscr.Identity();

	this->fov=fov;
	viewPlaneWidth=2.0f;
	viewPlaneHeight=2.0/aspectRatio;
	
	float tanFov_div2=tan(ANG_TO_RAD(fov/2));
	float cotFov_div2=1.0/tanFov_div2;
	viewDist=0.5*viewPlaneWidth*cotFov_div2;
	
	CPoint3D orign(0,0,0);
	if (fov==90.0)
	{
		CVector3 vn(1,0,-1);
		rtPlane.InitPlane3D(orign,vn,true);
		vn=CVector3(-1,0,-1);
		ltPlane.InitPlane3D(orign,vn,true);
		vn=CVector3(0,1,-1);
		tpPlane.InitPlane3D(orign,vn,true);
		vn=CVector3(0,-1,-1);
		btPlane.InitPlane3D(orign,vn,true);
	}
	else
	{
		CVector3 vn(viewDist,0,-viewPlaneWidth/2.0);
		rtPlane.InitPlane3D(orign,vn,TRUE);
		vn=CVector3(-viewDist,0,-viewPlaneWidth/2.0);
		ltPlane.InitPlane3D(orign,vn,TRUE);
		vn=CVector3(0,viewDist,-viewPlaneWidth/2.0);
		tpPlane.InitPlane3D(orign,vn,TRUE);
		vn=CVector3(0,-viewDist,-viewPlaneWidth/2.0);
		btPlane.InitPlane3D(orign,vn,TRUE);
	}

}

//////////////////////////////////////////////////////////////////////////
//�˺���������ʼ��ŷ���������
//Ҫ��������任������Ҫ�� ���ƽ�ƾ�����������xyz����ת����������
//////////////////////////////////////////////////////////////////////////

void CCamera::CreateCAM4DV1MatrixEuler(int rotateSeq)
{
	CMatrix4x4 mrot;           //���������ĳ˻�
	CMatrix4x4 mtemp;       

	//��1�����������λ�ü������ƽ�ƾ���������
	CMatrix4x4 mt_inv(1,0,0,0,0,1,0,0,0,0,1,0,
		-pos.x,-pos.y,-pos.z,1);        //���ƽ�ƾ���������

	//��2����������ת����������
	float thetaX=dir.x;
	float thetaY=dir.y;
	float thetaZ=dir.z;
	
	float cos_theta=FastCos(thetaX);  //cos(-x)=cos(x)
	float sin_theta=-FastSin(thetaX);   //sin(-x)=-sin(x)
	
	//�����x����ת����������
	CMatrix4x4 mx_inv(1,        0,         0, 0,
					  0,cos_theta, sin_theta, 0,
					  0,-sin_theta,cos_theta, 0,
				      0,        0,         0, 1);       

	cos_theta=FastCos(thetaY);  //cos(-x)=cos(x)
	sin_theta=-FastSin(thetaY);   //sin(-x)=-sin(x)

	//�����y����ת����������
	CMatrix4x4 my_inv(cos_theta,0,-sin_theta, 0,
					  0,        1,        0,  0,
				   	  sin_theta,0,cos_theta,  0,
					  0,        0,         0, 1);   

	cos_theta=FastCos(thetaZ);  //cos(-x)=cos(x)
	sin_theta=-FastSin(thetaZ);   //sin(-x)=-sin(x)

	//�����z����ת����������
	CMatrix4x4 mz_inv(cos_theta,sin_theta, 0, 0,
					  -sin_theta,cos_theta, 0, 0,
					          0,        0, 1, 0,
					          0,        0, 0, 1);   
	//��������ת����ĳ˻�
	switch (rotateSeq)
	{
	case CAM_ROTATE_XYZ:
		{
			mtemp=mx_inv*my_inv;
			mrot=mtemp*mz_inv;
		}break;
	case CAM_ROTATE_XZY:
		{
			mtemp=mx_inv*mz_inv;
			mrot=mtemp*my_inv;
		}break;
	case CAM_ROTATE_YXZ:
		{
			mtemp=my_inv*mx_inv;
			mrot=mtemp*mz_inv;
		}break;
	case CAM_ROTATE_YZX:
		{
			mtemp=my_inv*mz_inv;
			mrot=mtemp*mx_inv;
		}break;
	case CAM_ROTATE_ZXY:
		{
			mtemp=mz_inv*mx_inv;
			mrot=mtemp*my_inv;
		}break;
	case CAM_ROTATE_ZYX:
		{
			mtemp=mz_inv*my_inv;
			mrot=mtemp*mx_inv;
		}break;
	default:
		break;
	}
	mcam=mt_inv*mrot;
}

//////////////////////////////////////////////////////////////////////////
//�˺�������ע������n��������v��������u����һ������任����
//////////////////////////////////////////////////////////////////////////

void CCamera::CreateCAM4DV1MatrixUVN(int mode)
{
	//��1�����������λ�ô�����ƽ�ƾ���
	CMatrix4x4 mt_inv(1,0,0,0,
					  0,1,0,0,
					  0,0,1,0,
					  -pos.x,-pos.y,-pos.z,1);

	if (mode==UVN_MODE_SPHERICAL)
	{
		//ʹ����������ģʽ�����¼���Ŀ���
		//��ȡ��λ�Ǻ�����
		float phi=dir.x; //��λ��
		float theta=dir.y;  //����

		float sin_phi=FastSin(phi);
		float cos_phi=FastCos(phi);

		float sin_theta=FastSin(theta);
		float cos_theta=FastCos(theta);

		target.x= -cos_theta*sin_phi;
		target.y= sin_theta;
		target.z= cos_theta*cos_phi;
	}

	//1.n=<Ŀ��λ��-�۲�λ��>
	n.CreateVector(pos,target);

	//2.v=<0,1,0>
	v=CVector4(0,1,0);
	//3.u=(vxn)
	u=v.CrossProduct(n);

	//4.v=(nxu)
	v=n.CrossProduct(u);

	u.Normalize();
	v.Normalize();
	n.Normalize();
	
	//UVN����任����
	CMatrix4x4 mt_uvn(u.x,v.x,n.x,0,
					  u.y,v.y,n.y,0,
					  u.z,v.z,n.z,0,
					  0,0,0,1);            
	mcam=mt_inv*mt_uvn;
}