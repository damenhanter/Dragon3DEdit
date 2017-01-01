#pragma once

#include "datatype.h"
#include "Vector4.h"
#include "BitmapFile.h"
#include "BitmapImage.h"
#include "COBModel.h"

class CTerrain
{
public:
	CTerrain();
	~CTerrain();

	int GenerateTerrain(float twidth,float theight,float vscale,
		CPoint4D& pos,char* heightFieldFile,char* textMapFile,int rgbcolor,int Triattr);
public:
	void InitTerrain(int nVertices,int nTriangles);
	void ComputeOBJTriLength();
	void ComputeOBJVertexNormal();
public:
	int id;       //���������ID
	char name[256]; //������ַ����� 
	int state;
	int attr;
	int mati;
	
	int ivar1;
	int ivar2;
	float fvar1;
	float fvar2;

	CPoint4D worldPos;  //��������

	CVector4 dir;      //�����ھֲ�����ϵ�е���ת�Ƕ�

	CVector4 ux,uy,uz; //�ھֲ�����ϵ�еĳ���

	LPVERTEX vlocalList;  //�洢����ֲ����������,ָ��֡�����񶥵�ָ��
	LPVERTEX vtranslist;   //�洢�任��Ķ�����������,ָ��֡�����񶥵�ָ�루�任��İ汾��

	CPoint2D* textlist;  //���������б�
	CBitmapImage* ptrTexture;

	int nVertices;
	int nTriangles;       //��������Ķ������
	LPTri4DV1 plist;    //ָ������ָ��
};
