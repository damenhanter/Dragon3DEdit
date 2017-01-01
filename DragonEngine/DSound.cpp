#include "StdAfx.h"
#include "DSound.h"
extern HWND hWnd;
extern HINSTANCE hInstance;

CDSound::CDSound(void)
{
}

CDSound::~CDSound(void)
{
	ShutDown();
}

int CDSound::DSoundInit()
{
	static int flag=1;
	if(flag)
	{
		flag=0;
		if(FAILED(DirectSoundCreate(NULL,&lpds,NULL)))
		{
			MessageBox(hWnd,"DirectSound���󴴽�ʧ��!","error",MB_OK);
			return FALSE;
		}
		if(FAILED(lpds->SetCooperativeLevel(hWnd,DSSCL_NORMAL)))
		{
			MessageBox(hWnd,"����DirectSound��Э������!","error",MB_OK);
			return FALSE;
		}
		memset(sound,0,sizeof(PCMSound)*256);
	}

	for(int index=0; index<256; index++)
	{
		if(sound[index].dsbuffer!=NULL)
		{
			sound[index].dsbuffer->Stop();
			sound[index].dsbuffer->Release();
		}
		memset(&sound[index],0,sizeof(PCMSound));
		sound[index].state=SOUND_NULL;
		sound[index].id=index;
	}
	return TRUE;
}

int CDSound::DSoundLoadWave(char* filename,int controlflag)
{
	HMMIO hwav;
	MMCKINFO parent,child;   //�������ݿ�
	WAVEFORMATEX wfmtx;
	int soundId=-1;
	void* audioPtr1=NULL;
	void* audioPtr2=NULL;
	DWORD audioLength1=0;
	DWORD audioLength2=0;
	UCHAR* sendBufferPtr;
	for(int index=0; index<256; index++)
	{
		if(sound[index].state==SOUND_NULL)
		{
			soundId=index;
			break;
		}
	}
	if(soundId==-1)
		return FALSE;
	parent.ckid=(FOURCC)0;
	parent.cksize=0;
	parent.dwDataOffset=0;
	parent.dwFlags=0;
	parent.fccType=(FOURCC)0;

	child=parent;
	if((hwav=mmioOpen(filename,NULL,MMIO_ALLOCBUF|MMIO_READ))==NULL)
	{
		MessageBox(hWnd,"��wav�ļ�ʧ��!","error",MB_OK);
		return false;
	}
	parent.fccType=mmioFOURCC('W','A','V','E');
	if(mmioDescend(hwav,&parent,NULL,MMIO_FINDRIFF))
	{
		mmioClose(hwav,0);
		MessageBox(hWnd,"search next parent chunk failed!","error",MB_OK);
		return FALSE;
	}
	child.ckid=mmioFOURCC('f','m','t',' ');
	if(mmioDescend(hwav,&child,&parent,0))
	{
		mmioClose(hwav,0);
		MessageBox(hWnd,"search next child chunk failed!","error",MB_OK);
		return FALSE;
	}
	if(mmioRead(hwav,(char*)&wfmtx,sizeof(wfmtx))!=sizeof(wfmtx))
	{
		mmioClose(hwav,0);
		MessageBox(hWnd,"search wav file failed!","error",MB_OK);
		return false;
	}
	if(wfmtx.wFormatTag!=WAVE_FORMAT_PCM)
	{
		mmioClose(hwav,0);
		MessageBox(hWnd,"file is not wav!","error",MB_OK);
		return FALSE;
	}
	if(mmioAscend(hwav,&child,0))
	{
		mmioClose(hwav,0);
		MessageBox(hWnd,"ascend chunk an level is failed!","error",MB_OK);
		return FALSE;
	}
	child.ckid=mmioFOURCC('d','a','t','a');
	if(mmioDescend(hwav,&child,&parent,MMIO_FINDCHUNK))
	{
		mmioClose(hwav,0);
		MessageBox(hWnd,"search data is failed!","error",MB_OK);
		return FALSE;
	}
	sendBufferPtr=(UCHAR*)malloc(child.cksize);
	mmioRead(hwav,(char*)sendBufferPtr,child.cksize);
	mmioClose(hwav,0);
	sound[soundId].rate=wfmtx.nSamplesPerSec;
	sound[soundId].size=child.cksize;
	sound[soundId].state=SOUND_LOADED;

	memset(&pcmwf,0,sizeof(WAVEFORMATEX));
	pcmwf.wFormatTag=WAVE_FORMAT_PCM;
	pcmwf.nChannels=1;
	pcmwf.wBitsPerSample=8;
	pcmwf.nBlockAlign=1;     //pcmwf.nChannels*pcmwf.wBitsPerSample>>8;
	pcmwf.nSamplesPerSec=11025;
	pcmwf.nAvgBytesPerSec=pcmwf.nSamplesPerSec*pcmwf.nBlockAlign;
	pcmwf.cbSize=0;   //always 0;
	memset(&dsbd,0,sizeof(DSBUFFERDESC));

	dsbd.dwSize=sizeof(DSBUFFERDESC);
	dsbd.dwFlags=controlflag|DSBCAPS_LOCSOFTWARE|DSBCAPS_STATIC;
	dsbd.dwBufferBytes=child.cksize;
	dsbd.lpwfxFormat=&pcmwf;
	if(FAILED(lpds->CreateSoundBuffer(&dsbd,&sound[soundId].dsbuffer,NULL)))
	{
		free(sendBufferPtr);
		MessageBox(hWnd,"����DirectSound��������ʧ��!","error",MB_OK);
		return FALSE;
	}
	if(FAILED(sound[soundId].dsbuffer->Lock(0,child.cksize,&audioPtr1,&audioLength1,
		&audioPtr2,&audioLength2,DSBLOCK_FROMWRITECURSOR)))
	{
		MessageBox(hWnd,"��סDirectSound����ʧ��!","error",MB_OK);
		return FALSE;
	}
	CopyMemory(audioPtr1,sendBufferPtr,audioLength1);
	CopyMemory(audioPtr2,sendBufferPtr+audioLength1,audioLength2);
	
	sound[soundId].dsbuffer->Unlock(audioPtr1,audioLength1,audioPtr2,audioLength2);
	free(sendBufferPtr);
	return soundId;
}

int CDSound::play(int id, int flag)
{
	if(sound[id].dsbuffer!=NULL)
	{
		if(FAILED(sound[id].dsbuffer->SetCurrentPosition(0)))
		{
			MessageBox(hWnd,"���ò���λ��ʧ��!","error",MB_OK);
			return FALSE;
		}
		if(FAILED(sound[id].dsbuffer->Play(0,0,flag)))
		{
			MessageBox(hWnd,"��������ʧ��!","error",MB_OK);
			return FALSE;
		}
	}
	return TRUE;
}
/*����ֵ��ʵ��������˥��ֵ����ΧΪ-10000~0��0��ʾ
˥����С��ʱ�������-10000��-100db˥�������ʱ������С
vol��ȡֵ��ΧΪ0~100��
*/
int CDSound::SetVolume(int id,int vol)
{
	if(sound[id].dsbuffer!=NULL)
	{
		if(FAILED(sound[id].dsbuffer->SetVolume(DSVOLTODB(vol))))
		{
			MessageBox(hWnd,"��������ʧ��!","error",MB_OK);
			return FALSE;
		}
		else
			return TRUE;
	}
	else
		return FALSE;
}
/*Ƶ��--��λʱ���ڲ���������������ʹ�����������
������Խ������Խ��
*/
int CDSound::SetFrequence(int id,int frequ)
{
	if(sound[id].dsbuffer!=NULL)
	{
		if(FAILED(sound[id].dsbuffer->SetFrequency(frequ)))
		{
			MessageBox(hWnd,"����Ƶ��ʧ��!","error",MB_OK);
			return FALSE;
		}
		else
			return TRUE;
	}
	else
		return FALSE;
}
/*��Χ-100~100��0��ʾ�м�
-100��ʾ��������100��ʾ������
*/
int CDSound::SetPan(int id,long lpan)
{
	if(sound[id].dsbuffer!=NULL)
	{
		if(FAILED(sound[id].dsbuffer->SetPan(-lpan*100)))
		{
			MessageBox(hWnd,"��������ʧ��!","error",MB_OK);
			return FALSE;
		}
		else
			return TRUE;
	}
	else
		return FALSE;
}

int CDSound::StopSound(int id)
{
	if(sound[id].dsbuffer!=NULL)
	{
		if(FAILED(sound[id].dsbuffer->Stop()))
		{
			MessageBox(hWnd,"Stop����ʧ��!","error",MB_OK);
			return FALSE;
		}
		sound[id].dsbuffer->SetCurrentPosition(0);
	}
	return TRUE;
}
int CDSound::StopAllSound()
{
	for(int index=0; index<MAX_SOUNDS; index++)
		StopSound(index);
	return TRUE;
}
int CDSound::DeleteSound(int id)
{
	if(!StopSound(id))
		return FALSE;
	if(sound[id].dsbuffer!=NULL)
	{
		sound[id].dsbuffer->Release();
		sound[id].dsbuffer=NULL;
	}
	return TRUE;
}

int CDSound::DeleteAllSound()
{
	for(int index=0; index<MAX_SOUNDS; index++)
		DeleteSound(index);
	return TRUE;
}
DWORD CDSound::GetStatus(int id)
{
	DWORD status;
	if(sound[id].dsbuffer!=NULL)
	{
		sound[id].dsbuffer->GetStatus(&status);
		return status;
	}
	else
		return 0;
}
void CDSound::ShutDown()
{
	StopAllSound();
	DeleteAllSound();
	if(lpds!=NULL)
	{
		lpds->Release();
		lpds=NULL;
	}
	CoUninitialize();
}

int CDSound::ReplicateSound(int sourceId)
{
	if(sourceId!=-1)
	{
		for(int id=0; id<MAX_SOUNDS; id++)
		{
			if(sound[id].state==SOUND_NULL)
			{
				sound[id]=sound[sourceId];
				if(FAILED(lpds->DuplicateSoundBuffer(sound[sourceId].dsbuffer,&sound[id].dsbuffer)))
				{
					sound[id].state=SOUND_NULL;
					sound[id].dsbuffer=NULL;
					MessageBox(hWnd,"�ط�����ʧ��!","error",MB_OK);
					return FALSE;
				}
				sound[id].id=id;
				return id;
			}
			else
				return FALSE;
		}
	}
	else
		return FALSE;
	return TRUE;
}