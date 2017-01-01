#include "StdAfx.h"
#include "ZBuffer.h"

CZBuffer::CZBuffer(void)
{
	zbuffer=NULL;
}

CZBuffer::~CZBuffer(void)
{
	if (zbuffer!=NULL)
	{
		delete zbuffer;
		zbuffer=NULL;
	}
}

int CZBuffer::CreateZBuffer(int width, int height, int attr)
{
	if (zbuffer)
	{
		delete zbuffer;
	}

	this->width=width;
	this->height=height;
	this->attr=attr;
	//16λ����32λ
	if (attr & ZBUFFER_ATTR_16BIT)
	{
		sizeq=width*height/2;
		if ((zbuffer=new UCHAR[sizeq<<2])==NULL)
		{
			return FALSE;
		}
		else
		{
			return TRUE;
		}
	}
	else if (attr & ZBUFFER_ATTR_32BIT)
	{
		sizeq=width*height;
		if ((zbuffer=new UCHAR[sizeq<<3])==NULL)
		{
			return FALSE;
		}
		else
		{
			return TRUE;
		}
	}
	else
		return FALSE;
}

void CZBuffer::ClearZBuffer(UINT data)
{
	//���z������16λ�ģ�����ʹ����������16λֵ��������������һ����Ԫ��
	//���z������32λ�ģ�ֻ�轫��������ֵת��ΪUINT����
	MemSetDWORD(zbuffer,data,sizeq);
}