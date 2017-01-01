#pragma once
#include "datatype.h"
#include "BitmapFile.h"

class CBitmapImage
{
public:
	CBitmapImage(void);
	~CBitmapImage(void);
	CBitmapImage(CBitmapImage& image);
	CBitmapImage& operator=(CBitmapImage& image);

	int CreateBitmapImage(int x, int y, int width, int height, int bpp);
	int RenderBitmapImage(UCHAR* destBuffer, int lpitch, int transparent);
	int RenderBitmapImage16(UCHAR* destBuffer, int& lpitch, int transparent);
	int LoadImageBitmap(CBitmapFile& bitmap, int cx, int cy, int mode);
	int LoadImageBitmap16(CBitmapFile& bitmap, int cx, int cy, int mode);
	int CopyBitmapImage(int destX , int destY , CBitmapImage& sourceBitmap, int sourceX, int sourceY, int width, int height);
	int ScrollBitmap(int dx, int dy);
	UCHAR* buffer;      //λͼ����
	int width,height;   //λͼ��С
	int numBytes;       //λͼ�ֽ�
public:
	int state;          //λͼ״̬
	int attr;           //λͼ����
	int x,y;            //λͼλ��
	int bpp;
	


public:
	int Release(void);
};
