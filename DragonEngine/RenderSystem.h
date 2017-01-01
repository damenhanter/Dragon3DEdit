#pragma once
#include "datatype.h"
#include "BitmapFile.h"
#include "COBModel.h"
#include "terrain.h"
#include "Vector4.h"
#include "Matrix4x4.h"
#include "Camera.h"
#include "light.h"
#include "material.h"
#include "Line2D.h"
#include "ZBuffer.h"
#include "BSPTree.h"
extern HWND hWnd;

//�԰����ģ�����Ҫ���ⲿ���㣩����Σ�����Ⱦ�б�ʹ��
typedef struct TriF4DV1_TYPE
{
	int state;
	int attr;
	int color;

	int litColor[3];  //�洢���պ����ɫ��flat��ɫ�洢�������һ��ֵ��gouraud��ɫ������洢����ֵ
	CBitmapImage* ptrTexture;
	int mati;       //��������

	float nlength;  //�����η��߳���
	CVector4 normal;    //�����η���

	float avgZ;         //�������

	VERTEX vlist[3];  //�������ζ���
	VERTEX tvlist[3]; //�任��Ķ���

	TriF4DV1_TYPE* next;    //ָ����һ��������
	TriF4DV1_TYPE* prev;
}TriF4DV1,*LPTriF4DV1;

typedef struct RENDERLIST4DV1
{
	int state;
	int attr;

	//��Ⱦ�б���һ��ָ�����飬ÿ��ָ��ָ��һ���԰�����������
	LPTriF4DV1 triPtrs[MAX_RENDERLIST4DV1_TRIANGLES];
	//Ϊ����ÿ֡��Ϊ����η�����ͷŴ洢�ռ䣬����δ洢��triData��
	TriF4DV1 triData[MAX_RENDERLIST4DV1_TRIANGLES];

	int nTriangles;//�����������������
}RENDERLIST4DV1,*LPRENDERLIST4DV1;

//��Ⱦ����
typedef struct RENDERCONTEXT_TYP
{
	int attr;
	LPRENDERLIST4DV1 ptrRenderList;  //ָ��Ҫ��Ⱦ����Ⱦ�б��ָ��
	UCHAR* videoBuffer;      //ָ����Ƶ�����ָ��
	int lpitch;            //��Ƶ����Ŀ��

	UCHAR* zbuffer;    //ָ��z�����1/z�����ָ��
	int zpitch;    //z�����1/z����Ŀ��
	int alphaOverride;   //���������������ж���ε�alphaֵ

	int mipDist;  //���mip����
	int textDist; //ʹ��͸��/������ģʽʱ�����÷�������ӳ��ľ���
	
	//���Ժ���չʹ��
	int ival1,ival2;
	float fval1,fval2;
	void* vptr;
}RENDERCONTEXT,*LPRENDERCONTEXT;

class CRenderSystem
{
public:
	CRenderSystem(void);
	~CRenderSystem(void);

	USHORT (*RGB16Bit)(int r,int g,int b);

	int CreateDDraw(HWND hwnd,int screenWidth, int screenheight, int screenBPP,BOOL windowed);
	int DDrawFillSurface(LPDIRECTDRAWSURFACE7 lpdds,DWORD color);
	LPDIRECTDRAWSURFACE7 CreateSurface(int width, int height, int memflags, USHORT colorKeyValue);

	int FlipSurface(void);
	UCHAR* DDrawLockSurface(LPDIRECTDRAWSURFACE7 lpdds, int& lpitch);
	int DDrawUnLockSurface(LPDIRECTDRAWSURFACE7 lpdds);
	UCHAR* DDrawLockPrimarySurface();
	int DDrawUnLockPrimarySurface();
	UCHAR* DDrawLockBackSurface();
	int DDrawUnLockBackSurface();
	int GetClock(void);
	DWORD StartClock(void);
	DWORD WaitClock(DWORD count);
	int FillRectangle(int x1, int y1, int x2, int y2, int color, LPDIRECTDRAWSURFACE7 lpdds);
	
	int RotateColor(int startIndex, int endIndex);
	int BlinkColors(Blinker command, LPBLINKER newLight, int id);
	int ScanBitmapImage(CBitmapFile* lpbitmap, LPDIRECTDRAWSURFACE7 lpdds, int cx, int cy);
	int ScreenTransitions(ScreenAttr effect,UCHAR* vbuffer,int lpitch);
	
	int RenderRectangle(int sourceX1, int sourceY1, int sourceX2, int sourceY2, 
		int destX1, int destY1, int destX2, int destY2, 
		LPDIRECTDRAWSURFACE7 sourceSurface, LPDIRECTDRAWSURFACE7 destSurface);
	void LoadPaletteFromFile(char* filename);
	int SavePalette(LPPALETTEENTRY setPalette);
	int SetPalette(LPPALETTEENTRY setPalette);
	LPDIRECTDRAWSURFACE7 lpddsback;
	LPDIRECTDRAWSURFACE7 lpddsprimary;
	UCHAR* primaryBuffer;
	UCHAR* backBuffer;
	int primarylpitch;
	int backlpitch;
private:
	static inline USHORT RGB16Bit555(int r,int g,int b)
	{
		r>>=3; g>>=3; b>>=3;
		return (_RGB16BIT555(r,g,b));
	}
	static inline USHORT RGB16Bit565(int r,int g,int b)
	{
		r>>=3; g>>=2; b>>=3;
		return (_RGB16BIT565(r,g,b));
	}
	//static inline 
	int SetPaletteEntry(int colorIndex, LPPALETTEENTRY colorPalette);
	int GetPaletteEntry(int colorIndex, LPPALETTEENTRY colorPalette);
	
	void VLine(int y1, int y2, int x, int color, UCHAR* vbuffer, int lpitch);
	void VLine16(int y1, int y2, int x, int color, UCHAR* vbuffer, int lpitch);
	void HLine(int x1, int x2, int y, int color, UCHAR* vbuffer, int lpitch);
	void HLine16(int x1, int x2, int y, int color, UCHAR* vbuffer, int lpitch);
	void DDrawAttachClipper(LPDIRECTDRAWSURFACE7 lpdds, int numRects,LPRECT clipList);
public:
	void Release(void);
	void DrawPixel(int x,int y,int color);

public:
	
	void LoadTerrainModel(CTerrain* terrain,int insertLocal=true);

	void LoadRenderListFromModel(CCOBModel* ptrModel,BOOL insertLocal=0);

	inline void TranslateOBJ4DV1(CCOBModel* ptrModel,const CVector4& pos)
	{
		ptrModel->worldPos=ptrModel->worldPos+pos;
	}
	void ScaleOBJ4DV1(CCOBModel* ptrModel,const CVector4& scale,BOOL allFrames);

	inline void CreateRenderList()
	{
		ptrRenderList=new RENDERLIST4DV1;
		if (ptrRenderList==NULL)
		{
			MessageBox(hWnd,"CreateRenderlist failed,allocate failed","",MB_OK);
		}
		ptrRenderList->state=0;
		ptrRenderList->attr=0;
		ptrRenderList->nTriangles=0;
	}
	
	inline void ResetRenderList4DV1()
	{
		ptrRenderList->nTriangles=0;
	}
	void ResetOBJ4DV1(CCOBModel* ptrModel);
	
	void InsertTri4DV1ToRenderList4DV1(LPTri4DV1 triangle);
	void InsertTriF4DV1ToRenderList4DV1(LPTriF4DV1 triangle);
	
	void TransOBJModel4DV1(CCOBModel* ptrModel,const CMatrix4x4& mt,int type,BOOL tranbasis,BOOL allFrames);
	void TransRenderList4DV1(const CMatrix4x4& mt,int type);
	
	void CreateRotateMatrix4x4(float thetaX,float thetaY,float thetaZ,CMatrix4x4& mrot);

	void ModelToWorldOBJ4DV1(CCOBModel* ptrModel,int type=TRANSFORM_LOCAL_TO_TRANS,BOOL allFrames=FALSE);
	void CreateWorldMatrix4x4(CVector4& vpos,CMatrix4x4& Worldmatrix);
	void ModelToWorldRenderList4DV1(CVector4& Worldpos,int type=TRANSFORM_LOCAL_TO_TRANS);
	
	int CullOBJ4DV1(CCOBModel* ptrModel,const CCamera& cam,int cullFlag);
	void RemoveBackfaceOBJ4DV1(CCOBModel* ptrModel,const CCamera& cam);
	void RemoveBackfaceRenderList4DV1(const CCamera& cam);
	
	void WorldToCamOBJ4DV1(CCOBModel* ptrModel,const CCamera& cam);
	void WorldToCamRenderList4DV1(const CCamera& cam);
	
	void ClipTriangleRenderList4DV1(const CCamera& cam,int clipFlags);
	
	void TransLight4DV1(CLight& light,const CMatrix4x4& mt,int type);

	int LightOBJ4DV1World16(CCOBModel* ptrModel,const CCamera& camera,const CLight& light);
	int LightRenderList4DV1World16(const CCamera& CCamera,const CLight& light);

	void SortRenderList4DV1(int sortFlag);

	void CameraToPersOBJ4DV1(CCOBModel* ptrModel,const CCamera& cam);
	void CreateCameraToPersMatrix4x4(const CCamera& cam,CMatrix4x4& mt);
	void CameraToPersRenderList4DV1(const CCamera& cam);

	void PersToScreenOBJ4DV1(CCOBModel* ptrModel,const CCamera& cam);
	void CreatePersToScreen4DMatrix4x4(const CCamera&cam,CMatrix4x4& mt);
	void PersToScreenRenderList4DV1(const CCamera& cam);

	void ConvertToNotHomogeneous4DOBJ4DV1(CCOBModel* ptrModel);
	void ConvertToNotHomogeneous4DRenderList4DV1();

	void TraversalBSPInsertRENDERLIST(LPBSPNODE root,CCamera& camera,int insertLocal=TRUE);
	void TraversalBSPInsertRemoveBFRENDERLIST(LPBSPNODE root,CCamera& camera,int insertLocal=TRUE);
	void TraversalBSPInsertCullFrustrumRENDERLIST(LPBSPNODE root,CCamera& camera,int insertLocal=TRUE);

	void DrawOBJ4DV1Wire16(CCOBModel* ptrModel);
	void DrawRenderList4DV1Wire16();

	//z���治֧��alpha
	void DrawOBJ4DV1SolidZB16(CCOBModel* ptrModel,CZBuffer& zb);
	void DrawRenderList4DV1SolidZB16(CZBuffer& zb);
	//z����֧��alpha
	void DrawOBJ4DV1SolidZBAlpha16(CCOBModel* ptrModel,CZBuffer& zb,int alphaOverride);
	void DrawRenderList4DV1SolidZBAlpha16(CZBuffer& zb,int alphaOverride);
	//1/z���治֧��alpha
	void DrawOBJ4DV1SolidINVZB16(CCOBModel* ptrModel,CZBuffer& zb);
	void DrawRenderList4DV1SolidINVZB16(CZBuffer& zb);
	//1/z����֧��alpha
	void DrawOBJ4DV1SolidINVZBAlpha16(CCOBModel* ptrModel,CZBuffer& zb,int alphaOverride);
	void DrawRenderList4DV1SolidINVZBAlpha16(CZBuffer& zb,int alphaOverride);

	void DrawRenderListRENDERCONTEXT(LPRENDERCONTEXT rc);

public:
	int numModels;
	LPRENDERLIST4DV1 ptrRenderList;
private:
	LPDIRECTDRAW7 lpdd;
	LPDIRECTDRAWPALETTE lpddpal;
	LPDIRECTDRAWCLIPPER lpddclipper;          //���ڲü��󱸻���Ĳü���
	LPDIRECTDRAWCLIPPER lpddclipperwin;       //���ڲü�����
	DDSURFACEDESC2  ddsd;
	PALETTEENTRY palette[256];
	DDBLTFX ddbltfx;                          //Ӳ���������������Ϣ
	DDSCAPS2 ddscaps;
	
	int ddpixelformat;
	int screenWidth,screenHeight,screenBPP;
	BOOL windowed;
	DWORD startClockCount;
};
