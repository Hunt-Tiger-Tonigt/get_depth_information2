#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include<opencv2/opencv.hpp>
#include<iostream>

using namespace std;

void xian_de()
{
	cout << "��һЩû���õĳ���û���Ҿ����еģ�hhhhhh";
}

/*

bool mose_callback(int event, int x, int y, int flags, void*param);

cv::setMouseCallback("camera_left",mose_callback);

bool mose_callback(int event, int x, int y, int flags, void * param)
{
if (event == cv::EVENT_LBUTTONDOWN)
{
picture1 >> obtainl;
picture2 >> obtainr;
cout << "\n��⵽ţ�̺�" << jsq++ << "\n";
cv::imshow("������", obtainl);
cv::imshow("�����ң�", obtainr);
cv::imwrite("666.jpg", obtainl);
return true;
}
return false;
}
*/

/*
int i = 0;
for ( i ; i < mat.rows; i++)
{
continue;
}
fprintf(fp, "# .PCD v0.7 - Point Cloud Data file format\n");
fprintf(fp, "VERSION 0.7\n");
fprintf(fp, "FIELDS x y z\n");
fprintf(fp, "SIZE 4 4 4\n");
fprintf(fp, "TYPE F F F\n");
fprintf(fp, "COUNT 1 1 1\n");
fprintf(fp, "WIDTH %d \nHEIGHT 1\n", i);//�������Ҫ�������ǵ���
fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
fprintf(fp, "POINTS %d\n", i);
fprintf(fp, "DATA ascii\n");
WinExec("ren point_cloud.txt point_cloud.pcd", 0);
*/