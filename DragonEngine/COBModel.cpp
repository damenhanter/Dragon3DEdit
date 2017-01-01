#include "StdAfx.h"
#include "COBModel.h"
#include "Parser.h"
#include "Vector2.h"
#include "Matrix4x4.h"
#include "material.h"

extern UCHAR gLog2[513];

CCOBModel::CCOBModel(void)
{
	id=0;
	state=0;
	attr=0;
	numVertices=0;
	totalVertices=0;
	currFrame=0;
	numFrames=0;
	nTriangles=0;
	ptrTexture=NULL;
	vlocalList=NULL;
	vtranslist=NULL;
	vlocalListHead=NULL;
	vtranslistHead=NULL;
	plist=NULL;
	textlist=NULL;
	numMaterials=0;
}

CCOBModel::~CCOBModel(void)
{
	if (avgRadius)
	{
		delete [] avgRadius;
		avgRadius=NULL;
	}
	if (maxRadius)
	{
		delete [] maxRadius;
		maxRadius=NULL;
	}
	if (vlocalList)
	{
		delete [] vlocalList;
		vlocalList=NULL;
	}
	if (vtranslist)
	{
		delete [] vtranslist;
		vtranslist=NULL;
	}
	if (textlist)
	{
		delete [] textlist;
		textlist=NULL;
	}

	if (plist)
	{
		delete [] plist;
		plist=NULL;
	}

	if (attr & OBJMODEL4DV1_ATTR_MIPMAP)
	{
		DeleteMipMaps((CBitmapImage**)&ptrTexture);
	}
	else
	{
		if (ptrTexture)
		{
			delete ptrTexture;
			ptrTexture=NULL;
		}
	}
}
//////////////////////////////////////////////////////////////////////////
//numVertices:ÿ֡������ numTriangle:������� numFrames:֡��
//////////////////////////////////////////////////////////////////////////

int CCOBModel::InitModel(int numVertices, int numTriangles, int numFrames)
{
	if ((this->vlocalList=new VERTEX[numVertices*numFrames])==NULL)
	{
		return FALSE;
	}
	memset(this->vlocalList,0,sizeof(VERTEX)*numVertices*numFrames);

	if ((this->vtranslist=new VERTEX[numVertices*numFrames])==NULL)
	{
		return FALSE;
	}
	memset(this->vtranslist,0,sizeof(VERTEX)*numVertices*numFrames);

	if ((this->textlist=new CPoint2D[numTriangles*3])==NULL)
	{
		return FALSE;
	}
	memset(this->textlist,0,sizeof(CPoint2D)*numTriangles*3);

	if ((avgRadius=new float[numFrames])==NULL)
	{
		return FALSE;
	}
	memset(this->avgRadius,0,sizeof(float)*numFrames);

	if ((maxRadius=new float[numFrames])==NULL)
	{
		return FALSE;
	}
	memset(this->maxRadius,0,sizeof(float)*numFrames);

	if ((plist=new Tri4DV1[numTriangles])==NULL)
	{
		return FALSE;
	}
	memset(plist,0,sizeof(Tri4DV1)*numTriangles);

	vlocalListHead=vlocalList;
	vtranslistHead=vtranslist;

	this->numFrames=numFrames;
	this->nTriangles=numTriangles;
	this->numVertices=numVertices;
	this->totalVertices=numFrames*numVertices;

	return TRUE;
}

//�������ָ����֡����ĵ�ǰ֡��������Ƕ�֡���壬�˺�����Ч
void CCOBModel::SetFrame(int frame)
{
	if (!this)
	{
		return ;
	}
	if (attr & OBJMODEL4DV1_ATTR_MULTI_FRAME)
	{
		return ;
	}
	//������Ч���Ƕ�֡�ģ���ָ��ָ��ǰ֡����
	if (frame<0)
	{
		frame=0;
	}
	else if (frame >=numFrames)
	{
		frame=numFrames-1;
	}

	//���õ�ǰ֡
	currFrame=frame;

	//��ָ��ָ�򶥵��б�����Ӧ֡
	vlocalList=&vlocalListHead[frame*numVertices];
	vtranslist=&vtranslistHead[frame*numVertices];

}
//�������PLG��ƽ���뾶�����뾶
void CCOBModel::ComputeOBJModel4DV1Radius()
{
	avgRadius[currFrame]=0;
	maxRadius[currFrame]=0;
	float maxdist=0,dist=0;
	double totaldist=0.0f;
	for (int vertex=0;vertex<numVertices;vertex++)
	{
		dist=vlocalList[vertex].x*vlocalList[vertex].x
			+vlocalList[vertex].y*vlocalList[vertex].y
			+vlocalList[vertex].z*vlocalList[vertex].z;
		if (dist>maxdist)
		{
			maxdist=dist;
		}
		totaldist+=dist;
	}
	maxRadius[currFrame]=sqrt(maxdist);
	avgRadius[currFrame]=sqrt(totaldist/numVertices);
}

//////////////////////////////////////////////////////////////////////////
//�������η��ߵĳ���
//////////////////////////////////////////////////////////////////////////
void CCOBModel::ComputeOBJTriLength()
{
	for (int tri=0;tri<nTriangles;tri++)
	{
		int vert0=plist[tri].vert[0];
		int vert1=plist[tri].vert[1];
		int vert2=plist[tri].vert[2];
		CVector4 u,v,n;
		u.CreateVector(vlocalList[vert0].v,vlocalList[vert1].v);
		v.CreateVector(vlocalList[vert0].v,vlocalList[vert2].v);
		n=u.CrossProduct(v);
		plist[tri].nlength=n.GetLength();
	}
}

//////////////////////////////////////////////////////////////////////////
//�������嶥�㷨��
//////////////////////////////////////////////////////////////////////////

void CCOBModel::ComputeOBJVertexNormal()
{
	int triVertex[MAX_OBJMODEL4DV1_VERTICES];
	memset(triVertex,0,sizeof(int)*MAX_OBJMODEL4DV1_VERTICES);
	for (int tri=0;tri<nTriangles;tri++)
	{
		if (plist[tri].attr & TRI4DV1_ATTR_SHADE_MODE_GOURAUD)
		{
			int vert0=plist[tri].vert[0];
			int vert1=plist[tri].vert[1];
			int vert2=plist[tri].vert[2];
			triVertex[vert0]++;
			triVertex[vert1]++;
			triVertex[vert2]++;

			CVector4 u,v,n;
			u.CreateVector(vlocalList[vert0].v,vlocalList[vert1].v);
			v.CreateVector(vlocalList[vert0].v,vlocalList[vert2].v);
			n=u.CrossProduct(v);

			vlocalList[vert0].n=vlocalList[vert0].n+n;
			vlocalList[vert1].n=vlocalList[vert1].n+n;
			vlocalList[vert2].n=vlocalList[vert2].n+n;
		}
	}
	for (int vert=0;vert<numVertices;vert++)
	{
		if (triVertex[vert]>=1)
		{
			vlocalList[vert].nx/=triVertex[vert];
			vlocalList[vert].ny/=triVertex[vert];
			vlocalList[vert].nz/=triVertex[vert];

			vlocalList[vert].n.Normalize();
		}
	}
}
void CCOBModel::DeleteMipMaps(CBitmapImage** mipmaps,int level0)
{
	CBitmapImage** tmipmaps=(CBitmapImage**)*mipmaps;
	if (!tmipmaps)
	{
		return;
	}
	int numMipLevels=gLog2[tmipmaps[0]->width]+1;
	for (int mipLevel=1;mipLevel<numMipLevels;mipLevel++)
	{
		delete tmipmaps[mipLevel];
	}
	if (level0)
	{
		CBitmapImage* temp=tmipmaps[0];
	}
	else
	{
		delete tmipmaps[0];
	}
}
int CCOBModel::CreateMipmaps(CBitmapImage* source,CBitmapImage** mipmaps,float gamma)
{
	CBitmapImage** tmipmaps;  //ָ��ָ�������ָ��
	//1.����mip�ȼ�
	int numMipLevels=gLog2[source->width]+1;
	//Ϊָ����������ڴ�
	tmipmaps=(CBitmapImage**)malloc(numMipLevels*sizeof(CBitmapImage*));
	//��Ԫ��0ָ��ԭʼ����
	tmipmaps[0]=source;
	//���ÿ�Ⱥ͸߶�
	int mipWidth=source->width;
	int mipHeight=source->height;

	//ʹ��ƽ���˲������ɸ���mip����
	for (int mipLevel=1;mipLevel<numMipLevels;mipLevel++)
	{
		//������һ��mip����Ĵ�С
		mipWidth>>=1;
		mipHeight>>=1;
		//Ϊλͼ��������ڴ�

		tmipmaps[mipLevel]=new CBitmapImage;
		//�������ڴ洢mip�����λͼ
		tmipmaps[mipLevel]->CreateBitmapImage(0,0,mipWidth,mipHeight,16);
		//��λͼ��������Ⱦ
		SET_BIT(tmipmaps[mipLevel]->attr,BITMAP_ATTR_LOADED);
		//����ǰһ��mip����ʹ��ƽ���˲���������ǰmip����
		for(int x=0;x<tmipmaps[mipLevel]->width;x++)
		{
			for (int y=0;y<tmipmaps[mipLevel]->height;y++)
			{
				//��Ҫ����4������ƽ��ֵ����Щ������ǰһ��mip�����е�λ�ã�
				//(x*2,y*2) (x*2+1,y*2) (x*2,y*2+1) (x*2+1,y*2+1)
				//Ȼ�󽫼�����д��mip����x��y����
				float r0,g0,b0,r1,g1,b1,r2,g2,b2,r3,g3,b3;
				int r_avg,g_avg,b_avg;
				USHORT* scrBuffer=(USHORT*)tmipmaps[mipLevel-1]->buffer;
				USHORT* destBuffer=(USHORT*)tmipmaps[mipLevel]->buffer;
				//��ȡû�������RGBֵ
				_RGB565FROM16BIT(scrBuffer[(x*2+0)+(y*2+0)*mipWidth*2],&r0,&g0,&b0);
				_RGB565FROM16BIT(scrBuffer[(x*2+1)+(y*2+0)*mipWidth*2],&r1,&g1,&b1);
				_RGB565FROM16BIT(scrBuffer[(x*2+0)+(y*2+1)*mipWidth*2],&r2,&g2,&b2);
				_RGB565FROM16BIT(scrBuffer[(x*2+1)+(y*2+1)*mipWidth*2],&r3,&g3,&b3);

				//����ƽ��ֵ������gamma����
				r_avg=(int)(0.5f+gamma*(r0+r1+r2+r3)/4);
				g_avg=(int)(0.5f+gamma*(g0+g1+g2+g3)/4);
				b_avg=(int)(0.5f+gamma*(b0+b1+b2+b3)/4);

				if (r_avg>31)
				{
					r_avg=31;
				}
				if (g_avg>63)
				{
					g_avg=63;
				}
				if (b_avg>31)
				{
					b_avg=31;
				}
				destBuffer[x+y*mipWidth]=_RGB16BIT565(r_avg,g_avg,b_avg);
			}
		}
	}//
	*mipmaps=(CBitmapImage*)tmipmaps;
	return numMipLevels;
}

void CCOBModel::LoadModelFromFile(char* filename,CVector4& scale, CVector4& pos,CVector4& rotate,int VertexFlags)
{
	CParser parser;
	int nTextureVertex=0;
	CMatrix4x4 matLocal,matWorld;
	char texturePath[80]="..//res//";

	matLocal.Identity();
	matWorld.Identity();

	//1.��ʼ������
	state=OBJMODEL4DV1_STATE_ACTIVE|OBJMODEL4DV1_STATE_VISIBLE;
	worldPos=pos;
	currFrame=0;
	numFrames=1;
	attr=OBJMODEL4DV1_ATTR_SINGLE_FRAME;

	//2.���ļ�
	if (!parser.OpenFile(filename))
	{
		return ;
	}

	//3.��ȡ��������
	while(TRUE)
	{
		if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
		{
			return ;
		}
		
		if (parser.PatternMatch(parser.buffer,"['Name'] [s>0]"))
		{
			strcpy(name,parser.pstrings[1]);
			break;
		}
	}

	//4.��þֲ�������任����
	while(TRUE)
	{
		if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
		{
			return ;
		}
		if (parser.PatternMatch(parser.buffer,"['center'] [f] [f] [f]"))
		{
			matLocal.M[3][0]=-parser.pfloats[0];
			matLocal.M[3][1]=-parser.pfloats[1];
			matLocal.M[3][2]=-parser.pfloats[2];

			parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS);
			parser.PatternMatch(parser.buffer,"['x'] ['axis'] [f] [f] [f]");

			matLocal.M[0][0]=parser.pfloats[0];
			matLocal.M[1][0]=parser.pfloats[1];
			matLocal.M[2][0]=parser.pfloats[2];
			
			parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS);
			parser.PatternMatch(parser.buffer,"['y'] ['axis'] [f] [f] [f]");

			matLocal.M[0][1]=parser.pfloats[0];
			matLocal.M[1][1]=parser.pfloats[1];
			matLocal.M[2][1]=parser.pfloats[2];

			parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS);
			parser.PatternMatch(parser.buffer,"['z'] ['axis'] [f] [f] [f]");

			matLocal.M[0][2]=parser.pfloats[0];
			matLocal.M[1][2]=parser.pfloats[1];
			matLocal.M[2][2]=parser.pfloats[2];
			break;
			
		}
	}
	while(TRUE)
	{
		if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
		{
			return ;
		}
		if (parser.PatternMatch(parser.buffer,"['Transform']"))
		{
			parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS);
			parser.PatternMatch(parser.buffer,"[f] [f] [f] [f]");

			matWorld.M[0][0]=parser.pfloats[0];
			matWorld.M[1][0]=parser.pfloats[1];
			matWorld.M[2][0]=parser.pfloats[2];
			matWorld.M[3][0]=parser.pfloats[3];

			parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS);
			parser.PatternMatch(parser.buffer,"[f] [f] [f] [f]");

			matWorld.M[0][1]=parser.pfloats[0];
			matWorld.M[1][1]=parser.pfloats[1];
			matWorld.M[2][1]=parser.pfloats[2];
			matWorld.M[3][1]=parser.pfloats[3];

			parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS);
			parser.PatternMatch(parser.buffer,"[f] [f] [f] [f]");

			matWorld.M[0][2]=parser.pfloats[0];
			matWorld.M[1][2]=parser.pfloats[1];
			matWorld.M[2][2]=parser.pfloats[2];
			matWorld.M[3][2]=parser.pfloats[3];

			break;
			//��4�в���ȡ����0,0,0,1
		}
	}
	
	//6.��ö�������
	while(TRUE)
	{
		if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
		{
			return ;
		}
		if (parser.PatternMatch(parser.buffer,"['World'] ['Vertices'] [i]"))
		{
			numVertices=parser.pints[0];
			break;
		}
	}
	InitModel(numVertices,numVertices*3,numFrames);

	//7.���ض����б�
	for(int vert=0; vert<numVertices;vert++)
	{
		while(true)
		{
			if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
			{
				return ;
			}
			if (parser.PatternMatch(parser.buffer,"[f] [f] [f]"))
			{
				vlocalList[vert].x=parser.pfloats[0];
				vlocalList[vert].y=parser.pfloats[1];
				vlocalList[vert].z=parser.pfloats[2];
				vlocalList[vert].w=1;

				if (VertexFlags & VERTEX_FLAGS_TRANSFORM_LOCAL)
				{
					vlocalList[vert].v=vlocalList[vert].v*matLocal;
				}
				if (VertexFlags & VERTEX_FLAGS_TRANSFORM_LOCAL_WORLD)
				{
					vlocalList[vert].v=vlocalList[vert].v*matWorld;
				}
				if (VertexFlags & VERTEX_FLAGS_INVERT_X)
				{
					vlocalList[vert].x=-vlocalList[vert].x;
				}
				if (VertexFlags & VERTEX_FLAGS_INVERT_Y)
				{
					vlocalList[vert].y=-vlocalList[vert].y;
				}
				if (VertexFlags & VERTEX_FLAGS_INVERT_Z)
				{
					vlocalList[vert].z=-vlocalList[vert].z;
				}
				if (VertexFlags & VERTEX_FLAGS_SWAP_YZ)
				{
					swap(vlocalList[vert].y,vlocalList[vert].z);
				}
				if (VertexFlags & VERTEX_FLAGS_SWAP_XZ)
				{
					swap(vlocalList[vert].x,vlocalList[vert].z);
				}
				if (VertexFlags & VERTEX_FLAGS_SWAP_XY)
				{
					swap(vlocalList[vert].x,vlocalList[vert].y);
				}

				vlocalList[vert].x*=scale.x;
				vlocalList[vert].y*=scale.y;
				vlocalList[vert].z*=scale.z;

				SET_BIT(vlocalList[vert].attr,VERTEX_ATTR_POINT);
				break;
			}
		}
	}
	
	ComputeOBJModel4DV1Radius();

	//8.������������Ŀ
	while(TRUE)
	{
		if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
		{
			return ;
		}
		if (parser.PatternMatch(parser.buffer,"['Texture'] ['Vertices'] [i]"))
		{
			nTextureVertex=parser.pints[0];
			break;
		}
	}
	
	for (int i=0;i<nTextureVertex;i++)
	{
		while(TRUE)
		{
			if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
			{
				return ;
			}
			if (parser.PatternMatch(parser.buffer, "[f] [f]"))
			{
				textlist[i].x=parser.pfloats[0];
				textlist[i].y=parser.pfloats[1];
				break;
			}
		}
	}

	//10.���ض�����б�
	int TriMaterial[MAX_OBJMODEL4DV1_TRIANGLES];
	int MaterialIndexRef[MAX_MATERIALS];       //���ڼ��������õĲ����������������������
	memset(MaterialIndexRef,0,sizeof(MaterialIndexRef));

	while(TRUE)
	{
		if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
		{
			return ;
		}
		if (parser.PatternMatch(parser.buffer,"['Faces'] [i]"))
		{
			nTriangles=parser.pints[0];
			break;
		}
	}
	int desc=0;
	int numMaterialObjects=0;
	for (int tri=0;tri<nTriangles;tri++)
	{
		while(true)
		{
			if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
			{
				return ;
			}
			if (parser.PatternMatch(parser.buffer,"['Face'] ['verts'] [i] ['flags'] [i] ['mat'] [i]"))
			{
				TriMaterial[tri]=parser.pints[2];
				if (MaterialIndexRef[TriMaterial[tri]]==0)
				{
					MaterialIndexRef[TriMaterial[tri]]=1;
					numMaterialObjects++;
				}
				if (parser.pints[0]!=3)
				{
					return ;
				}
				if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
				{
					return ;
				}
				parser.ReplaceChars(parser.buffer,parser.buffer,",<>",' ');
				parser.PatternMatch(parser.buffer,"[i] [i] [i] [i] [i] [i]");

				if (VertexFlags & VERTEX_FLAGS_INVERT_WINDING_ORDER)
				{
					plist[tri].vert[0]=parser.pints[4];
					plist[tri].vert[1]=parser.pints[2];
					plist[tri].vert[2]=parser.pints[0];

					plist[tri].text[0]=parser.pints[5];
					plist[tri].text[1]=parser.pints[3];
					plist[tri].text[2]=parser.pints[1];

				}
				else
				{
					plist[tri].vert[0]=parser.pints[0];
					plist[tri].vert[1]=parser.pints[2];
					plist[tri].vert[2]=parser.pints[4];

					plist[tri].text[0]=parser.pints[1];
					plist[tri].text[1]=parser.pints[3];
					plist[tri].text[2]=parser.pints[5];
				}
				plist[tri].vlist=vlocalList;
				plist[tri].textlist=textlist;

				plist[tri].state=TRI4DV1_STATE_ACTIVE;
				break;
			}
		}
	}

	for (int currMat=0;currMat<numMaterialObjects;currMat++)
	{
		while(true)
		{
			if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
			{
				return ;
			}
			if (parser.PatternMatch(parser.buffer, "['mat#'] [i]"))
			{
				int matIndex=parser.pints[0];
				while(true)
				{
					if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
					{
						return ;
					}
					parser.ReplaceChars(parser.buffer,parser.buffer,",", ' ');
					if (parser.PatternMatch(parser.buffer,"['rgb'] [f] [f] [f]"))
					{
						Materials[matIndex+numMaterials].color.r=(UCHAR)(parser.pfloats[0]*255+0.5);
						Materials[matIndex+numMaterials].color.g=(UCHAR)(parser.pfloats[1]*255+0.5);
						Materials[matIndex+numMaterials].color.b=(UCHAR)(parser.pfloats[2]*255+0.5);
						break;
					}
				}
				while(true)
				{
					if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
					{
						return ;
					}
					if (parser.PatternMatch(parser.buffer,"['alpha'] [f] ['ka'] [f] ['ks'] [f] ['exp'] [f]"))
					{
						Materials[matIndex+numMaterials].color.a=(UCHAR)(parser.pfloats[0]*255+0.5);
						Materials[matIndex+numMaterials].ka=parser.pfloats[1];
						Materials[matIndex+numMaterials].ks=parser.pfloats[2];
						Materials[matIndex+numMaterials].kd=1;
						Materials[matIndex+numMaterials].power=parser.pfloats[3];

						//������ǰ�洢�Ĳ�����ɫ
						for (int i=1;i<=3;i++)
						{
							Materials[matIndex+numMaterials].ra.M[i]=Materials[matIndex+numMaterials].ka*Materials[matIndex+numMaterials].color.M[i]+0.5;
							Materials[matIndex+numMaterials].rd.M[i]=Materials[matIndex+numMaterials].kd*Materials[matIndex+numMaterials].color.M[i]+0.5;
							Materials[matIndex+numMaterials].rs.M[i]=Materials[matIndex+numMaterials].ks*Materials[matIndex+numMaterials].color.M[i]+0.5;
						}
						break;
					}
				}
				while(true)
				{
					if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
					{
						return ;
					}
					if (parser.PatternMatch(parser.buffer,"['Shader'] ['class:'] ['color']"))
					{
						break;
					}
				}
				while (true)
				{
					if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
					{
						return ;
					}
					parser.ReplaceChars(parser.buffer,parser.buffer, "\"", ' ');
					if (parser.PatternMatch(parser.buffer,"['Shader'] ['name:'] ['plain'] ['color']"))
					{
						break;
					}
					if (parser.PatternMatch(parser.buffer,"['Shader'] ['name:'] ['texture'] ['map']"))
					{
						SET_BIT(Materials[matIndex+numMaterials].attr,MATV1_ATTR_SHADE_MODE_TEXTURE);
						while(true)
						{
							if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
							{
								return ;
							}
							parser.ReplaceChars(parser.buffer, parser.buffer, "\"", ' ');
							if (parser.PatternMatch(parser.buffer,"['file'] ['name:'] ['string']"))
							{
								memcpy(Materials[matIndex+numMaterials].texturePath,&parser.buffer[18],
									strlen(parser.buffer)-18+2);
								if (!ptrTexture)
								{
									ptrTexture=new CBitmapImage;
									char fileName[80];
									char filePath[80];
									parser.GetFileNameFromPath(Materials[matIndex+numMaterials].texturePath,fileName);
									strcpy(filePath,texturePath);
									strcat(filePath,fileName);

									CBitmapFile bitmap16file;
									bitmap16file.LoadBitmapFromFile(filePath);
									ptrTexture->CreateBitmapImage(0,0,bitmap16file.bitmapinfoheader.biWidth,
										bitmap16file.bitmapinfoheader.biHeight,bitmap16file.bitmapinfoheader.biBitCount);
									ptrTexture->LoadImageBitmap16(bitmap16file,0,0,BITMAP_EXTRACT_MODE_ABS);
									bitmap16file.UnLoadBitmapFile();

									SET_BIT(attr,OBJMODEL4DV1_ATTR_TEXTURES);
								}
								break;
							}
						}
						break;
					}
				}
//////////////////////////////////////////////////////////////////////////
//����͸����alpha
//////////////////////////////////////////////////////////////////////////
				while(true)
				{
					if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
					{
						return ;
					}
					if (parser.PatternMatch(parser.buffer,"['Shader']['class:']['transparency']"))
					{
						break;
					}
				}
				while (true)
				{
					if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
					{
						return;
					}
					parser.ReplaceChars(parser.buffer,parser.buffer,"\"",' ',1);
					if (parser.PatternMatch(parser.buffer,"['Shader'] ['name:'] [s>0]"))
					{
						if (strcmp(parser.pstrings[2],"none")==0)
						{
							RESET_BIT(Materials[matIndex+numMaterials].attr,MATV1_ATTR_TRANSPARENT);
							//��alpha�ȼ�����Ϊ0��
							Materials[matIndex+numMaterials].color.a=0;
						}
						else if (strcmp(parser.pstrings[2],"filter")==0)
						{
							SET_BIT(Materials[matIndex+numMaterials].attr,MATV1_ATTR_TRANSPARENT);
							while(true)
							{
								if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES|PARSER_STRIP_WS_ENDS))
								{
									return;
								}
								parser.ReplaceChars(parser.buffer,parser.buffer,":(,)",' ',1);
								if (parser.PatternMatch(parser.buffer,"['colour']['color'][i][i][i]"))
								{
									//��alpha�ȼ�����ΪRGB������
									int maxAlpha=max(parser.pints[0],parser.pints[1]);
									maxAlpha=max(maxAlpha,parser.pints[2]);
									Materials[matIndex+numMaterials].color.a=
										(int)((float)maxAlpha/255*(float)(NUM_ALPHA_LEVELS-1)+(float)0.5);
									if (Materials[matIndex+numMaterials].color.a>=NUM_ALPHA_LEVELS)
									{
										Materials[matIndex+numMaterials].color.a=NUM_ALPHA_LEVELS-1;
									}
									break;
								}
							}
						}
						break;
					}
				}
//////////////////////////////////////////////////////////////////////////
				while(true)
				{
					if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
					{
						return ;
					}
					if (parser.PatternMatch(parser.buffer,"['Shader'] ['class:'] ['reflectance']"))
					{
						break;
					}
				}

				while(true)
				{
					if (!parser.GetLine(PARSER_STRIP_EMPTY_LINES | PARSER_STRIP_WS_ENDS))
					{
						return ;
					}
					parser.ReplaceChars(parser.buffer, parser.buffer, "\"", ' ');
					if (parser.PatternMatch(parser.buffer,"['Shader'] ['name:'] [s>0]"))
					{
						if (strcmp(parser.pstrings[2],"constant")==0)
						{
							SET_BIT(Materials[matIndex+numMaterials].attr,MATV1_ATTR_SHADE_MODE_CONSTANT);
						}
						else if (strcmp(parser.pstrings[2],"matte")==0)
						{
							SET_BIT(Materials[matIndex+numMaterials].attr,MATV1_ATTR_SHADE_MODE_FLAT);
						}
						else if (strcmp(parser.pstrings[2],"plastic")==0)
						{
							SET_BIT(Materials[matIndex+numMaterials].attr,MATV1_ATTR_SHADE_MODE_GOURAUD);
						}
						else if (strcmp(parser.pstrings[2],"phong")==0)
						{
							SET_BIT(Materials[matIndex+numMaterials].attr,MATV1_ATTR_SHADE_MODE_FASTPHONG);
						}
						else
						{
							SET_BIT(Materials[matIndex+numMaterials].attr,MATV1_ATTR_SHADE_MODE_FLAT);
						}	
						break;
					}
				}
				break;
			}
		}
	}
//////////////////////////////////////////////////////////////////////////
	if (VertexFlags & USE_MIPMAP)
	{
		SET_BIT(attr,OBJMODEL4DV1_ATTR_MIPMAP);
		CreateMipmaps(ptrTexture,(CBitmapImage**)&ptrTexture,1.01f);
	}
//////////////////////////////////////////////////////////////////////////
	for (int tri=0;tri<nTriangles;tri++)
	{
		SET_BIT(plist[tri].attr,TRI4DV1_ATTR_RGB16);

		if (plist[tri].attr & MATV1_ATTR_SHADE_MODE_TEXTURE)
		{
			plist[tri].color=_RGB16BIT565(255,255,255);
		}
		else
		{
			plist[tri].color=_RGB16BIT565(Materials[TriMaterial[tri]].color.r,Materials[TriMaterial[tri]].color.g,
				Materials[TriMaterial[tri]].color.b);
		}
		
		if (Materials[TriMaterial[tri]].attr & MATV1_ATTR_SHADE_MODE_CONSTANT)
		{
			SET_BIT(plist[tri].attr,TRI4DV1_ATTR_SHADE_MODE_CONSTANT);
		}
		else if (Materials[TriMaterial[tri]].attr & MATV1_ATTR_SHADE_MODE_FLAT)
		{
			SET_BIT(plist[tri].attr,TRI4DV1_ATTR_SHADE_MODE_FLAT);
		}
		else if (Materials[TriMaterial[tri]].attr & MATV1_ATTR_SHADE_MODE_GOURAUD)
		{
			SET_BIT(plist[tri].attr,TRI4DV1_ATTR_SHADE_MODE_GOURAUD);
			SET_BIT(vlocalList[plist[tri].vert[0]].attr,VERTEX_ATTR_NORMAL);
			SET_BIT(vlocalList[plist[tri].vert[1]].attr,VERTEX_ATTR_NORMAL);
			SET_BIT(vlocalList[plist[tri].vert[2]].attr,VERTEX_ATTR_NORMAL);
		}
		else if (Materials[TriMaterial[tri]].attr & MATV1_ATTR_SHADE_MODE_FLAT)
		{
			SET_BIT(plist[tri].attr,TRI4DV1_ATTR_SHADE_MODE_FLAT);
		}
		else if (Materials[TriMaterial[tri]].attr & MATV1_ATTR_SHADE_MODE_FASTPHONG)
		{
			SET_BIT(plist[tri].attr,TRI4DV1_ATTR_SHADE_MODE_FASTPHONG);
			SET_BIT(vlocalList[plist[tri].vert[0]].attr,VERTEX_ATTR_NORMAL);
			SET_BIT(vlocalList[plist[tri].vert[1]].attr,VERTEX_ATTR_NORMAL);
			SET_BIT(vlocalList[plist[tri].vert[2]].attr,VERTEX_ATTR_NORMAL);
		}
		else
		{
			SET_BIT(plist[tri].attr,TRI4DV1_ATTR_SHADE_MODE_GOURAUD);
		}

		if (Materials[TriMaterial[tri]].attr & MATV1_ATTR_SHADE_MODE_TEXTURE)
		{
			SET_BIT(plist[tri].attr,TRI4DV1_ATTR_SHADE_MODE_TEXTURE);
			plist[tri].ptrTexture=ptrTexture;
			
			if (VertexFlags & USE_MIPMAP)
			{
				SET_BIT(plist[tri].attr,TRI4DV1_ATTR_MIPMAP);
			}

			SET_BIT(vlocalList[plist[tri].vert[0]].attr,VERTEX_ATTR_TEXTURE);
			SET_BIT(vlocalList[plist[tri].vert[1]].attr,VERTEX_ATTR_TEXTURE);
			SET_BIT(vlocalList[plist[tri].vert[2]].attr,VERTEX_ATTR_TEXTURE);
		}

		if (Materials[TriMaterial[tri]].attr & MATV1_ATTR_TRANSPARENT)
		{
			plist[tri].color+=(Materials[TriMaterial[tri]].color.a<<24);
			SET_BIT(plist[tri].attr,TRI4DV1_ATTR_TRANSPARENT);
		}
		//
		SET_BIT(plist[tri].attr,TRI4DV1_ATTR_DISABLE_MATERIAL);
		//
	}
	numMaterials+=numMaterialObjects;

	if (ptrTexture)
	{
		for (int i=0;i<nTextureVertex;i++)
		{
			int size;
			if (VertexFlags & USE_MIPMAP)
			{
				size=((CBitmapImage**)(ptrTexture))[0]->width;
			}
			else
			{
				size=ptrTexture->width;
			}
			
			textlist[i].x*=(size-1);
			textlist[i].y*=(size-1);
			if (VertexFlags & VERTEX_FLAGS_INVERT_TEXTURE_U)
			{
				textlist[i].x=(size-1)-textlist[i].x;
			}
			if (VertexFlags & VERTEX_FLAGS_INVERT_TEXTURE_V)
			{
				textlist[i].y=(size-1)-textlist[i].y;
			}
			if (VertexFlags & VERTEX_FLAGS_INVERT_SWAP_UV)
			{
				swap(textlist[i].x,textlist[i].y);
			}
		}
	}

	LPTri4DV1 tempTri=new Tri4DV1[nTriangles];
	memcpy(tempTri,plist,sizeof(Tri4DV1)*nTriangles);
	delete [] plist;
	plist=tempTri;

	ComputeOBJTriLength();
	ComputeOBJVertexNormal();
}
