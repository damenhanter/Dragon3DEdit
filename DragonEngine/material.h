#pragma once
#include "datatype.h"
#include "BitmapImage.h"

class CMaterial
{
public:
	CMaterial();
	~CMaterial();
	
public:
	int state;
	int id;
	char name[64];
	int attr;

	RGBAV1 color;      //������ɫ
	float ka,kd,ks,power;    //����ϵ���;��淴��ָ��

	RGBAV1 ra,rd,rs;      //��ǰ�õ�����ɫ�ͷ���ϵ���Ļ�
	char texturePath[80];    //����ͼ·��
	CBitmapImage texture;        //����ͼ
};