#pragma once
#include "BitmapFile.h"
#include "BitmapImage.h"
#include "Vector4.h"
#include "COBModel.h"

typedef struct PolyF4DV1_TYP
{
	int state;
	int attr;
	int color;   //多边形颜色
	int litColor[4];   //用于存储光照后的颜色
	CBitmapImage* ptrTexture;
	int mati;

	float nlength;
	CVector4 n;
	float avgZ;
	VERTEX vlist[4];  //四边形顶点
	VERTEX tvlist[4];
	
	PolyF4DV1_TYP* next;
	PolyF4DV1_TYP* prev;
}PolyF4DV1,*LPPolyF4DV1;

typedef struct BSPNode_TYP
{
	int id;  //用于调试
	PolyF4DV1 wall;

	BSPNode_TYP* link;  //指向下一个墙面指针 
	BSPNode_TYP* front;
	BSPNode_TYP* back;
}BSPNODE,*LPBSPNODE;

class CBSPTree
{
public:
	CBSPTree(void);
	~CBSPTree(void);

	void IntersectLines(float x0,float y0,float x1,float y1,
		float x2,float y2,float x3,float y3,float& xi,float& yi);
	void SetUp(LPBSPNODE* tempRoot);
	void TransformBSP(LPBSPNODE root,CMatrix4x4& mt,int coord_select=TRANSFORM_LOCAL_TO_TRANS);
	void DeleteBSP(LPBSPNODE* root);
public:
	
};
