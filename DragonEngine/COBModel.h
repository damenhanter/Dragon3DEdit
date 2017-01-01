#pragma once
#include "datatype.h"
#include "material.h"
#include "BitmapFile.h"
#include "BitmapImage.h"
#include "Vector2.h"
#include "Vector3.h"
#include "Vector4.h"

typedef struct VERTEX_TYPE
{
	VERTEX_TYPE() { memset(this, 0, sizeof(VERTEX_TYPE)); }
	~VERTEX_TYPE() {}

	VERTEX_TYPE& operator=(const VERTEX_TYPE& it)
	{
		memcpy(this->M, it.M, sizeof(VERTEX_TYPE));
		return *this;
	}
	union
	{
		struct
		{
			float x,y,z,w;
			float nx,ny,nz,nw;
			float u0,v0;       //��������

			float i;   //���ն�������
			int attr;
		};
		float M[12];
		struct
		{
			CPoint4D v;  //����
			CVector4 n;  //����Gouraud��ɫ��Ҫ֪�����㷨��
			CPoint2D t;  //��������
			float i;
			int attr;
		};
	};
}VERTEX,*LPVERTEX;

typedef struct Tri4DV1_TYPE
{
	int state;     //״̬��Ϣ
	int attr;       //��������
	int color;      //��ɫ

	int litColor[3];  //�洢���պ����ɫ��flat��ɫ�洢�������һ��ֵ��gouraud��ɫ������洢����ֵ
	CBitmapImage* ptrTexture;
	int mati;       //��������

	LPVERTEX vlist;   // �����б�
	CPoint2D* textlist; //�������궥������ 
	int vert[3];       //��������
	int text[3];       //����������
	float nlength;  //���߳���
}Tri4DV1,*LPTri4DV1;

class CCOBModel
{
public:
	CCOBModel(void);
	~CCOBModel(void);

	int InitModel(int numVertices,int numTriangles,int numFrames);
	void ComputeOBJModel4DV1Radius();
	void SetFrame(int frame);
	void ComputeOBJTriLength();
	void ComputeOBJVertexNormal();
	void LoadModelFromFile(char* filename,CVector4& scale,
		CVector4& pos,CVector4& rotate,int VertexFlags=false);
	int CreateMipmaps(CBitmapImage* source,CBitmapImage** mipmaps,float gamma);
	void DeleteMipMaps(CBitmapImage** mipmaps,int level0=true);
public:
	int id;			//���������ID
	char name[256]; //������ַ����� 
	int state;
	int attr;
	int mati;

	float* avgRadius;   //ƽ���뾶
	float* maxRadius;   //���뾶
	//���������֡��ƽ���뾶�����뾶�治ͬ��֡��ͬ����ͬ

	CPoint4D worldPos;  //��������

	CVector4 dir;      //�����ھֲ�����ϵ�е���ת�Ƕ�

	CVector4 ux,uy,uz; //�ھֲ�����ϵ�еĳ���

	int numVertices;     //����ÿ֡�Ķ�����
	int totalVertices;   //�����ܹ�������
	int numFrames;     //�ܹ��ж���֡
	int currFrame;     //��ǰ֡

	LPVERTEX vlocalList;  //�洢����ֲ����������,ָ��֡�����񶥵�ָ��
	LPVERTEX vtranslist;   //�洢�任��Ķ�����������,ָ��֡�����񶥵�ָ�루�任��İ汾��

	LPVERTEX vlocalListHead;
	LPVERTEX vtranslistHead;

	CPoint2D* textlist;  //���������б�
	CBitmapImage* ptrTexture;

	int nTriangles;       //��������Ķ������
	LPTri4DV1 plist;    //ָ������ָ��

	int numMaterials;
	CMaterial Materials[MAX_MATERIALS];
};
