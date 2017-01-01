#include "StdAfx.h"
#include "Parser.h"
#include "ErrorLog.h"
extern CErrorLog errorlog;
#pragma warning (disable:4018)

CParser::CParser(void)
{
	fstream=NULL;
	Reset();
}

CParser::~CParser(void)
{
	Reset();
}

int CParser::Reset()
{
	if (fstream)
	{
		fclose(fstream);
		fstream=NULL;
	}
	memset(buffer,0,sizeof(buffer));
	length=0;
	numLines=0;
	strcpy(comment,"#");
	return TRUE;
}

int CParser::OpenFile(char *filename)
{
	Reset();
	if ((fstream=fopen(filename,"r"))==NULL)
	{
		return FALSE;
	}
	return TRUE;
}

int CParser::CloseFile()
{
	return (Reset());
}

char* CParser::GetLine(int mode)
{
	char* string;
	//���ļ����л�ȡ����
	if (fstream)
	{
		if (mode & PARSER_STRIP_EMPTY_LINES)
		{
			while (true)
			{
				//�ļ��Ƿ�Ϊ��
				if ((string=fgets(buffer,PARSER_BUFFER_SIZE,fstream))==NULL)
				{
					break;
				}
				int length=strlen(string);
				int index=0;

				while(isspace(string[index]))
					index++;
				//����Ƿ��пո�
				if ((length-index)>0)
				{
					memmove(buffer,&string[index],length-index+1);
					string=buffer;
					length=strlen(string);

					if (mode & PARSER_STRIP_COMMENTS)
					{
						char* comment_string=strstr(string,comment);
						if (comment_string==NULL)
						{
							break;
						}
						int index=int(comment_string-string);
						if (index=0)
						{
							continue;
							//����ע��,���ԣ���һ��
						}
						else
						{
							comment_string[index]=0;
							break;
							//�޳�ע�ͷ�
						}
					}
					break;
				}
			}
		}
		else
		{
			string=fgets(buffer,PARSER_BUFFER_SIZE,fstream);
		}

		if (string)
		{
			numLines++;

			if (mode & PARSER_STRIP_WS_ENDS)
			{
				StringLTrim(buffer);
				StringRTrim(buffer);
			}
			length=strlen(buffer);
			return string;
		}
		else
		{
			return NULL;
		}
	}
	else
	{
		return FALSE;
	}
	
}

int CParser::SetComment(char* string)
{
	if (strlen(string)<PARSER_MAX_COMMENT)
	{
		strcpy(comment,string);
		return true;
	}
	else
	{
		return FALSE;
	}
}

//////////////////////////////////////////////////////////////////////////
//����ַ����Ƿ����ת��Ϊ����
//��ת���ĸ�ʽ
// [whitespace] [sign]digits
//////////////////////////////////////////////////////////////////////////
int CParser::IsInt(char* istring)
{
	char* string=istring;
	
	//[whitespace] 
	while (isspace(*string))
	{
		string++;
	}

	//[sign]
	if (*string=='+' || *string=='-')
	{
		string++;
	}

	//digits
	while(isdigit(*string))
	{
		string++;
	}

	if (strlen(istring)==(int)(string-istring))
	{
		return (atoi(istring));
	}
	else
		return INT_MIN;
}

//////////////////////////////////////////////////////////////////////////
//����ַ����Ƿ����ת��Ϊ������
//��ת���ĸ�ʽ
// [whitespace] [sign] [digits] [.digits] [ {d | D | e | E }[sign]digits]
//////////////////////////////////////////////////////////////////////////
float CParser::IsFloat(char* fstring)
{
	char* string=fstring;
	
	//[whitespace]
	while(isspace(*string))
	{
		string++;
	}

	//[sign]
	if (*string=='+'||*string=='-')
	{
		string++;
	}

	//[digits]
	while(isdigit(*string))
	{
		string++;
	}

	//[.digits]
	if (*string=='.')
	{
		string++;
		while(isdigit(*string))
		{
			string++;
		}
	}
	
	// [ {d | D | e | E }[sign]digits]
	if (*string=='d' || *string=='D' || *string=='e' || *string=='E')
	{
		string++;
		if (*string=='+'||*string=='-')
		{
			string++;
		}
		while(isdigit(*string))
		{
			string++;
		}
	}	

	if (strlen(fstring)==(int)(string-fstring))
	{
		return atof(fstring);
	}
	else
	{
		return FLT_MIN;
	}
}

char* CParser::StringRTrim(char *string)
{
	int index;
	int length=strlen(string);
	if (!string ||length<=0)
	{
		return NULL;
	}
	index=length-1;
	while(isspace(string[index])&&index>=0)
	{
		string[index]=0;
		index--;
	}
	return string;
}

char* CParser::StringLTrim(char* string)
{
	int index=0;
	int length=strlen(string);
	if (!string||length<=0)
	{
		return NULL;
	}
	while(isspace(string[index]&& index<length))
	{
		string[index++]=0;
	}
	memmove(string,&string[index],int(length-index+1));
	return string;
}

int CParser::ReplaceChars(char *stringIn, char *stringOut, char *replaceChars, char repChar, int CaseFlag)
{
	int numReplace=0;
	int indexIn=0,indexOut=0;
	int index=0,length=0;

	if (!stringIn||!stringOut||strlen(stringIn)==0)
	{
		return NULL;
	}

	if (!replaceChars||strlen(replaceChars)==0)
	{
		strcpy(stringOut,stringIn);
		return FALSE;
	}
	
	length=strlen(replaceChars);
	if (CaseFlag)
	{
		while(stringIn[indexIn])
		{
			for (index=0;index<length;index++)
			{
				if (stringIn[indexIn]==replaceChars[index])
				{
					stringOut[indexOut++]=repChar;
						indexIn++;
					numReplace++;
					break;
				}
			}
			if (index>=length)
			{
				stringOut[indexOut++]=stringIn[indexIn++];
			}
		}
	}
	else
	{
		while(stringIn[indexIn])
		{
			for (index=0;index<length;index++)
			{
				if (toupper(stringIn[indexIn])==toupper(replaceChars[index]))
				{
					stringOut[indexOut++]=repChar;
					indexIn++;
					numReplace++;
					break;
				}
			}
			if (index>=length)
			{
				stringOut[indexOut++]=stringIn[indexIn++];
			}
		}
	}
	stringOut[indexOut]=0;
	return numReplace;
}

int CParser::StripChars(char *stringIn, char *stringOut, char *stripChars, int CaseFlag)
{
	int numExtract=0;
	int indexIn=0,indexOut=0;
	int index=0,length=0;

	if (!stringIn||!stringOut||strlen(stringIn)==0)
	{
		return NULL;
	}

	if (!stripChars||strlen(stripChars)==0)
	{
		strcpy(stringOut,stringIn);
		return FALSE;
	}

	length=strlen(stripChars);
	if (CaseFlag)
	{
		while(stringIn[indexIn])
		{
			for (index=0;index<length;index++)
			{
				if (stringIn[indexIn]==stripChars[index])
				{
					//ֱ�Ӵ�stringIn������
					indexIn++;
					numExtract++;
					break;
				}
			}
			if (index>=length)
			{
				stringOut[indexOut++]=stringIn[indexIn++];
			}
		}
	}
	else
	{
		while(stringIn[indexIn])
		{
			for (index=0;index<length;index++)
			{
				if (toupper(stringIn[indexIn])==toupper(stripChars[index]))
				{
					indexIn++;
					numExtract++;
					break;
				}
			}
			if (index>=length)
			{
				stringOut[indexOut++]=stringIn[indexIn++];
			}
		}
	}
	stringOut[indexOut]=0;
	return numExtract;
}

int CParser::PatternMatch(char* string,char* pattern,...)
{
	char tokenType[PATTERN_MAX_ARGS];              //�洢�Ǻ�����
	char tokenString[PATTERN_MAX_ARGS][PATTERN_BUFFER_SIZE];   //�洢�ַ�������
	char tokenOperator[PATTERN_MAX_ARGS];	          //�洢�ɱ䳤���ַ����Ĳ�����
	int tokenNumberic[PATTERN_MAX_ARGS];                //�洢�ɱ䳤���ַ�������
	char buffer[PATTERN_BUFFER_SIZE];

	if ((!string || strlen(string)==0)  || (!pattern || strlen(pattern)==0))
	{
		return FALSE;
	}
	strcpy(buffer,string);

	int start=0;
	int end=0;
	int restart=0;
	int firstPass=0;
	int numTokens=0;
	
	//��һ������ȡ�Ǻ��б�
	while(TRUE)
	{
		while(isspace(pattern[start]))
		{
			start++;
		}
		if (start>=strlen(pattern))
		{
			break;
		}

   		if (pattern[start]=='[')
		{
			switch (pattern[start+1])
			{
			case PATTERN_TOKEN_FLOAT:
				{
					if (pattern[start+2]!=']')
					{
						return FALSE;
					}
					start+=3;
					tokenType[numTokens]=PATTERN_TOKEN_FLOAT;
					strcpy(tokenString[numTokens],"");
					tokenOperator[numTokens]=0;
					tokenNumberic[numTokens]=0;
					numTokens++;
				}break;
			case PATTERN_TOKEN_INT:
				{
					if (pattern[start+2]!=']')
					{
						return FALSE;
					}
					start+=3;
					tokenType[numTokens]=PATTERN_TOKEN_INT;
					strcpy(tokenString[numTokens],"");
					tokenOperator[numTokens]=0;
					tokenNumberic[numTokens]=0;
					numTokens++;
				}break;
			case PATTERN_TOKEN_LITERAL:
				{
					start+=2;
					end=start;

					while(pattern[end]!=PATTERN_TOKEN_LITERAL)
					{
						end++;
					}
					if (pattern[end+1]!=']')
					{
						return FALSE;
					}
					memcpy(tokenString[numTokens],&pattern[start],end-start);
					tokenString[numTokens][end-start]=0;
					tokenType[numTokens]=PATTERN_TOKEN_LITERAL;
					tokenOperator[numTokens]=0;
					tokenNumberic[numTokens]=0;

					start=end+2;
					numTokens++;
				}break;
			case PATTERN_TOKEN_STRING:
				{
					if (pattern[start+2]=='='||
						pattern[start+2]=='>'||
						pattern[start+2]=='<')
					{
						end=start+3;
						while(isdigit(pattern[end]))
						{
							end++;
						}
						if (pattern[end]!=']')
						{
							return FALSE;
						}
						memcpy(buffer,&pattern[start+3],(end-(start)));
						buffer[end-start]=0;

						tokenType[numTokens]=PATTERN_TOKEN_STRING;
						strcpy(tokenString[numTokens],"");
						tokenOperator[numTokens]=pattern[start+2];
						tokenNumberic[numTokens]=atoi(buffer);
					}
					else
					{
						return FALSE;
					}
					start=end+1;
					numTokens++;
				}break;
			default:
				break;
			}
		}
		if (start>=strlen(pattern))
		{
			break;
		}
	}
	//״̬��
	int patternState=PATTERN_STATE_INIT;
	int currToken=0;
	char token[PATTERN_BUFFER_SIZE];
	while(TRUE)
	{
		switch(patternState)
		{
		case PATTERN_STATE_INIT:
			{
				strcpy(buffer,string);
				start=0;
				end=0;
				restart=0;
				firstPass=1;
				currToken=0;

				num_pfloats=num_pints=num_pstrings=0;
				patternState=PATTERN_STATE_RESTART;
			}break;
		case  PATTERN_STATE_RESTART:
			{
				currToken=0;
				firstPass=1;
			
				//errorlog.WriteError("%s\n",buffer);
				//������
				if (currToken>=strlen(buffer))
				{
					return FALSE;
				}

				start=end=restart;
				patternState=PATTERN_STATE_NEXT;

			}break;
		case PATTERN_STATE_NEXT:
			{
				if (currToken>=numTokens)
				{
					//ƥ�����
					patternState=PATTERN_STATE_MATCH;
				}
				else
				{
					//������
					if (end>=strlen(buffer))
					{
						return FALSE;
					}
					//��ȡ�ո�ǰ��ƥ���ַ���
					start=end;
					while(isspace(buffer[start]))
					{
						start++;
					}
					end=start;
					while(!isspace(buffer[end]) && end<strlen(buffer))
					{
						end++;
					}
					memcpy(token,&buffer[start],end-start);
					token[end-start]=0;
					//
					if (strlen(token)==0)
					{
						return false;
					}
					//�����һ�θû�ȡ�ַ�����λ��
					if (firstPass)
					{
						firstPass=0;
						restart=end;
					}
					//���ݵ�ǰ�Ǻ����ͣ�������Ҫ������
					switch (tokenType[currToken])
					{
					case PATTERN_TOKEN_FLOAT:
						{
							patternState=PATTERN_STATE_FLOAT;
						}break;
					case PATTERN_TOKEN_INT:
						{
							patternState=PATTERN_STATE_INT;
						}break;
					case PATTERN_TOKEN_LITERAL:
						{
							patternState=PATTERN_STATE_LITERAL;
						}break;
					case PATTERN_TOKEN_STRING:
						{
							patternState=PATTERN_STATE_STRING;
						}break;
					default:
						break;
					}
				}
			}break;
		case PATTERN_STATE_FLOAT:
			{
				float f=IsFloat(token);
				if (f!=FLT_MIN)
				{
					pfloats[num_pfloats++]=f;
					currToken++;
					patternState=PATTERN_STATE_NEXT;
				}
				else
				{
					patternState=PATTERN_STATE_RESTART;
				}
			}break;
		case PATTERN_STATE_INT:
			{
				float i=IsInt(token);
				if (i!=INT_MIN)
				{
					pints[num_pints++]=i;
					currToken++;
					patternState=PATTERN_STATE_NEXT;
				}
				else
				{
					patternState=PATTERN_STATE_RESTART;
				}
			}break;
		case PATTERN_STATE_LITERAL:
			{
				if (strcmp(token,tokenString[currToken])==0)
				{
					strcpy(pstrings[num_pstrings++],token);
					currToken++;
					patternState=PATTERN_STATE_NEXT;
				}
				else
				{
					patternState=PATTERN_STATE_RESTART;
				}
			}break;
		case PATTERN_STATE_STRING:
			{
				switch (tokenOperator[currToken])
				{
				case '=':
					{
						if (strlen(token)==tokenNumberic[currToken])
						{
							strcpy(pstrings[num_pstrings++],token);
							currToken++;
							patternState=PATTERN_STATE_NEXT;
						}
						else
						{
							patternState=PATTERN_STATE_RESTART;
						}

					}break;
				case '>':
					{
						if (strlen(token)>tokenNumberic[currToken])
						{
							strcpy(pstrings[num_pstrings++],token);
							currToken++;
							patternState=PATTERN_STATE_NEXT;
						}
						else
						{
							patternState=PATTERN_STATE_RESTART;
						}

					}break;
				case '<':
					{
						if (strlen(token)<tokenNumberic[currToken])
						{
							strcpy(pstrings[num_pstrings++],token);
							currToken++;
							patternState=PATTERN_STATE_NEXT;
						}
						else
						{
							patternState=PATTERN_STATE_RESTART;
						}

					}break;
				default:
					break;
				}
			}break;
		case PATTERN_STATE_MATCH:
			{
				return true;
			}break;
		case PATTERN_STATE_END:
			{

			}break;
		}
	}
}
int CParser::GetFileNameFromPath(char *filePath, char *fileName)
{
	if (!filePath || strlen(filePath)==0)
	{
		return false;
	}
	int end=strlen(filePath)-1;
	while( (filePath[end]!='\\')&&(filePath[end]!='/') && (end>0) )
	{
		end--;
	}
	memcpy(fileName,&filePath[end+1],strlen(filePath)-end);
	return true;
}