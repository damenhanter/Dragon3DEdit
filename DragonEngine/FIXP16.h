#pragma once
#include "datatype.h"

class CFIXP16
{
public:
	CFIXP16(void);
	CFIXP16(int x)
	{
		this->x=x;
	}

	CFIXP16(CFIXP16& src)
	{
		x=src.x;
	}
	~CFIXP16(void);

	CFIXP16 operator=(const CFIXP16& src)
	{
		x=src.x;
	}
	inline CFIXP16 operator+(const CFIXP16& fp)
	{
		return CFIXP16(x+fp.x);
	}
	inline CFIXP16 operator-(const CFIXP16& fp)
	{
		return CFIXP16(x-fp.x);
	}
	inline CFIXP16 operator*(const CFIXP16& fp)
	{
		CFIXP16 result;
		int temp=fp.x;
		_asm
		{
			mov eax,x
			imul temp
			shrd eax,edx,16
			mov  result.x,eax
		}
		//��ʽ��   shrd   Ŀ�Ĳ�������Դ����������λλ�� 
		//ָ��ı�Դ��������Դ����������Ϊ�Ĵ�����
		//��Ŀ�Ĳ����������ƶ���Ӧλ�����ճ�����λ��Դ�������ĵ�λ��䡣
		return result;
	}
	inline CFIXP16 operator/(const CFIXP16& fp)
	{
		CFIXP16 result;
		int temp=fp.x;
		_asm
		{
			mov eax,x
			cdq        //��������չ��edx:eax��
			shld edx,eax,16   //��edx������λ
			sal eax,16
			idiv temp
			mov result.x,eax
		}
		return result;
	}
	inline void IntegerToFixp16(int i)
	{
		x=(i)<<FIXP16_SHIFT;
	}
	inline void FloatToFixp16(float f)
	{
		x=int(f*FIXP16_MAG+0.5);
	}
	//��������ת��Ϊ������
	inline float Fixp16ToFloat()
	{
		return ((float)x)/FIXP16_MAG;
	}
	//��16.16��ʽ�Ķ���������ȡ�������ֺ�С������
	inline int GetWP()
	{
		return (x)>>FIXP16_SHIFT;
	}
	inline int GetDP()
	{
		return (x)&&FIXP16_DP_MASK;
	}
public:
	int x;
};
