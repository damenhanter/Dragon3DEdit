#pragma once
#include "datatype.h"

class CParser
{
public:
	CParser(void);
	~CParser(void);

	int Reset();
	int OpenFile(char* filename);
	int CloseFile();
	char* GetLine(int mode);
	int SetComment(char* string);
	int PatternMatch(char* string,char* pattern,...);
	
	//��������
	int IsInt(char* istring);
	float IsFloat(char* fstring);
	char* StringRTrim(char* string);
	char* StringLTrim(char* string);
	int ReplaceChars(char* stringIn,char* stringOut,char* replaceChars,char repChar,int CaseFlag=1);
	int StripChars(char* stringIn,char* stringOut,char* stripChars,int CaseFlag=1);
	int GetFileNameFromPath(char* filePath,char* fileName);
public:
	FILE* fstream;                        //�ļ�ָ��
	char buffer[PARSER_BUFFER_SIZE];      // ������
	int length;                           //��ǰ�г�
	int numLines;                         //�����˶�����
	char comment[PARSER_MAX_COMMENT];     //����ע���ַ���

	//ģʽƥ������洢�ռ�
	//����PatternMatch()����ʱ��ƥ������ݽ��洢����Щ������
	char pstrings[PATTERN_MAX_ARGS][PATTERN_BUFFER_SIZE];    //�ַ���
	int  num_pstrings;
	float pfloats[PATTERN_MAX_ARGS];                   //������
	int num_pfloats;
	int pints[PATTERN_MAX_ARGS];                //����
	int num_pints;
};
