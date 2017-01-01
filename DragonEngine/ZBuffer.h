#pragma once

#include "datatype.h"

class CZBuffer
{
public:
	CZBuffer(void);
	~CZBuffer(void);

	int CreateZBuffer(int width,int height,int attr);
	void ClearZBuffer(UINT data);
public:
	int attr;    //����
	UCHAR* zbuffer;  //ָ�򻺴�ռ��ָ��
	int width;       //��ȣ���λ����
	int height;      //�߶�
	int sizeq;       //��С����λΪ��Ԫ��
};
