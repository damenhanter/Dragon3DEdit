#pragma once
#include "datatype.h"

class CBitmapFile
{
public:
	CBitmapFile(void);
	~CBitmapFile(void);

	int LoadBitmapFromFile(char* filename);
	int FlipBitmap(UCHAR* image, int BytesPerLine, int height);
public:
	BITMAPFILEHEADER bitmapfileheader;      //������λͼ�ļ�ͷ����Ϣ
	BITMAPINFOHEADER bitmapinfoheader;      //���������е�λͼ��Ϣ
	PALETTEENTRY palette[256];              //���λͼ�����ǰ�λ���洢��ɫ��
	UCHAR* buffer;                          //����ָ��
	int UnLoadBitmapFile(void);
};
