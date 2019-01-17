#include <opencv2/opencv.hpp>
#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <windows.h>
#include<iostream>

using namespace std;

int CountLines(const char* filename)
{
	ifstream ReadFile;
	int n = 0;
	string tmp;
	ReadFile.open(filename, ios::in);//ios::in 表示以只读的方式读取文件
	if (ReadFile.fail())//文件打开失败:返回0
	{
		return 0;
	}
	else//文件存在
	{
		while (getline(ReadFile, tmp, '\n'))
		{
			n++;
		}
		ReadFile.close();
		return n;
	}
}