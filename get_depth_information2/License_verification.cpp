#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <windows.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

#pragma warning(disable:4267)

// 获取Beauty.exe所在路径
void getExePath(char *path, int size)
{
	char szBuf[1025] = { 0 };
	GetModuleFileName(NULL, szBuf, sizeof(szBuf));
	char *p = strrchr(szBuf, '\\');
	*p = '\0';
	strncpy(path, szBuf, size - 1);
	path[size - 1] = '\0';
}

// 私有转化算法（不能公开）
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


// Beauty.exe的开始部分
int License_verification()
{

	/* Beauty.exe软件的认证部分， 如果不通过， 则不可以用Beauty.exe */

	// 获取Beauty.exe所在目录下的license.txt文件的第一行
	char szLicenseFile[2049] = { 0 };
	getExePath(szLicenseFile, sizeof(szLicenseFile));
	strcat(szLicenseFile, "\\license.txt"); // strcat不安全哈
	// 校验license.txt文件是否存在
	ifstream in(szLicenseFile);
	if (!in)
	{
		cout << "无license.txt文件， 认证失败" << endl;
		cout << "\n" << "请联系管理员获取许可证";
		cout << "\n" << "联系方式：";
		cout << "\n" << "QQ：136768916";
		cout << "\n" << "Phone：18398621916";
		while (1);
		return -1;
	}
	// 获取并校验license.txt文件第一行
	string line;
	getline(in, line);
	if (line == string(""))
	{
		cout << "license.txt文件错误" << endl;
		while (1);
		return -1;
	}
	// 获取license.txt文件指纹
	char szLine[1025] = { 0 };
	strncpy(szLine, line.c_str(), sizeof(szLine) - 1);
	// 获取运行Beauty.exe的PC的指纹
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
	cout << "您的序列号为：\n" << szFingerPrint << "\n";
	// 私有变换
	privateConvert(szFingerPrint);
	// 判断license.txe文件指纹和PC指纹是否一致
	if (0 != strcmp(szLine, szFingerPrint))
	{
		cout << "license.txt文件认证失败" << endl;
		cout << "\n" << "请联系管理员获取许可证";
		cout << "\n" << "联系方式：";
		cout << "\n" << "QQ：136768916";
		cout << "\n" << "Phone：18398621916";
		while (1);
		return -1;
	}
	// 软件的功能部分， 此处略去10000行功能性语句
	cout << "欢迎使用本软件" << endl;
	return 1;
}