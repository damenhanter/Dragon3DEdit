#pragma once
#include "datatype.h"
#include "Line2D.h"

class CPolygon2D
{
public:
	CPolygon2D(void);
	CPolygon2D(int state, int numVerts, POINT centerPoint,VERTEX2D* pVertex, int Vx, int Vy, int color);
	~CPolygon2D(void);
public:
	int DrawPolygon2D(UCHAR* vbuffer, int lpitch);
	int DrawPolygon162D(UCHAR* vbuffer, int lpitch);
	int TranslatePolygon2D(int dx, int dy);	// ƽ�ƶ����
	int RotatePolygon2D(int angle);
	int ScalePolygon2D(float sx, float sy);
	int DrawFillPolygon2D(UCHAR* vbuffer, int lpitch);	// ��������
	int DrawFillPolygon162D(UCHAR* vbuffer, int lpitch);
public:
	LPVERTEX2D m_pVertex;
	int m_NumVerts;	           //��������
	int m_State;               //�������ʾ״̬
	POINT m_CenterPoint;       //�����λ��
	int m_Vx;                  // X�����ٶ�
	int m_Vy;                  // Y �����ٶ�
	DWORD m_Color;
	CLine2D line2D;
};
