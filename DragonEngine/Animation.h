#pragma once
#include "datatype.h"
#include "RenderSystem.h"
#include "BitmapFile.h"
#include "BitmapImage.h"

class CAnimation
{
public:
	CAnimation(void);
	~CAnimation(void);
public:
	int CreateAnimation(CRenderSystem* render,float x, float y, int width, int height, int numFrames, int attr, 
		int memFlags, USHORT colorKeyValue, int bpp);
	int CloneAnim(CAnimation* lpSourceAnim);
	int RenderAnimation(CRenderSystem* render,LPDIRECTDRAWSURFACE7 destSurface);
	int RenderScaleAnim(CRenderSystem* render,int swidth, int sheight, LPDIRECTDRAWSURFACE7 destSurface);
	int LoadAnimFrame(CBitmapFile* bitmap, int frame, int cx, int cy, int mode);
	int LoadAnimFrame16(CBitmapFile* bitmap, int frame, int cx, int cy, int mode);
	int DoAnimation(void);
	int Move(void);
	int LoadAnimation(int anim, int numFrames, int* sequence);
	int SetPos(float x, float y);
	int SetAnimSpeed(int speed);
	int SetAnimation(int anim);
	int SetVelocity(float xv, float yv);
	int Hide(void);
	int Show(void);
	int CollisionCheck(CAnimation* anim);
	int Release(void);
public:
	int state;         //����״̬
	int animState;    //����״̬
	int attr;          //��������
	float x,y;         //λͼ��ʾ��λ��
	int bpp;
	float xv,yv;       //�����ٶ�
	int width,height;  //������
	int counter1;
	int counter2;
	int maxCounter1;
	int maxCounter2;
	int varsI[16];      //16λ����ջ
	float varsF[16];    //16λ������ջ
	int currFrame;      //��ǰ֡
	int numFrames;      //������֡��
	int currAnimation;   //��ǰ��������
	int animCounter;    //�������㶯��ת��
	int animIndex;      //����Ԫ������
	int animCountMax;   
	int* animations[ANIM_MAX_ANIMATIONS];  //��������
public:
	LPDIRECTDRAWSURFACE7 images[ANIM_MAX_FRAMES];   //����λͼ�ı���

};
