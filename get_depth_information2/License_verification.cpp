#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <windows.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

#pragma warning(disable:4267)

// ��ȡBeauty.exe����·��
void getExePath(char *path, int size)
{
	char szBuf[1025] = { 0 };
	GetModuleFileName(NULL, szBuf, sizeof(szBuf));
	char *p = strrchr(szBuf, '\\');
	*p = '\0';
	strncpy(path, szBuf, size - 1);
	path[size - 1] = '\0';
}

// ˽��ת���㷨�����ܹ�����
void privateConvert(char *pStr)
{
	int len = strlen(pStr);
	int i = 0;
	for (i = 0; i < len; i++)
	{
		pStr[i] = pStr[i] * pStr[i] % 128; 
		if ('\0' == pStr[i])
		{
			pStr[i] = 1;
		}
	}
}


// Beauty.exe�Ŀ�ʼ����
int License_verification()
{

	/* Beauty.exe�������֤���֣� �����ͨ���� �򲻿�����Beauty.exe */

	// ��ȡBeauty.exe����Ŀ¼�µ�license.txt�ļ��ĵ�һ��
	char szLicenseFile[2049] = { 0 };
	getExePath(szLicenseFile, sizeof(szLicenseFile));
	strcat(szLicenseFile, "\\license.txt"); // strcat����ȫ��
	// У��license.txt�ļ��Ƿ����
	ifstream in(szLicenseFile);
	if (!in)
	{
		cout << "��license.txt�ļ��� ��֤ʧ��" << endl;
		cout << "\n" << "����ϵ����Ա��ȡ���֤";
		cout << "\n" << "��ϵ��ʽ��";
		cout << "\n" << "QQ��136768916";
		cout << "\n" << "Phone��18398621916";
		while (1);
		return -1;
	}
	// ��ȡ��У��license.txt�ļ���һ��
	string line;
	getline(in, line);
	if (line == string(""))
	{
		cout << "license.txt�ļ�����" << endl;
		while (1);
		return -1;
	}
	// ��ȡlicense.txt�ļ�ָ��
	char szLine[1025] = { 0 };
	strncpy(szLine, line.c_str(), sizeof(szLine) - 1);
	// ��ȡ����Beauty.exe��PC��ָ��
	const int MAX_BUFFER_LEN = 500;
	char  szBuffer[MAX_BUFFER_LEN];
	char  szBuffer_1[MAX_BUFFER_LEN];
	DWORD dwNameLen;
	dwNameLen = MAX_BUFFER_LEN;
	GetComputerName(szBuffer, &dwNameLen);
	int v = 0;
	int a = strlen(szBuffer);
	for (int i = 0; i < a; i++)
	{
		v |= ((unsigned int)szBuffer[a - 1 - i] & 0xFFu) << (i * 8);
	}
	int u = 0;
	GetUserName(szBuffer_1, &dwNameLen);
	int b = strlen(szBuffer_1);
	for (int i = 0; i < b; i++)
	{
		u |= ((unsigned int)szBuffer_1[b - 1 - i] & 0xFFu) << (i * 8);
	}
	char szFingerPrint[1025];
	char v1[1025];
	_itoa(u, szFingerPrint, 10);
	_itoa(v, v1, 10);
	strcat(szFingerPrint, v1);
	cout << "�������к�Ϊ��\n" << szFingerPrint << "\n";
	// ˽�б任
	privateConvert(szFingerPrint);
	// �ж�license.txe�ļ�ָ�ƺ�PCָ���Ƿ�һ��
	if (0 != strcmp(szLine, szFingerPrint))
	{
		cout << "license.txt�ļ���֤ʧ��" << endl;
		cout << "\n" << "����ϵ����Ա��ȡ���֤";
		cout << "\n" << "��ϵ��ʽ��";
		cout << "\n" << "QQ��136768916";
		cout << "\n" << "Phone��18398621916";
		while (1);
		return -1;
	}
	// ����Ĺ��ܲ��֣� �˴���ȥ10000�й��������
	cout << "��ӭʹ�ñ����" << endl;
	return 1;
}