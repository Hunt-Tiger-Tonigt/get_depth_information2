/*******************************
*  @Function   get_depth_information
*  @Works      获取目标图像位姿
*  @Author     Hunt Tiger Tonight
*  @Platform   VS2015 C++
*  @Connect    phone：18398621916/QQ:136768916
*  @Date       2019-01-16
********************************/

#include <F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <string>
#include <windows.h>
#include <Mmsystem.h>
#include <cstring>
#include <shellapi.h>
#include <tchar.h>  
#include <fstream> 
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h> 

#pragma comment(lib, "shell32.lib")
#pragma comment(lib,"winmm.lib")
#pragma warning(disable:4101)

using namespace std;

#define pi 3.1415926535

//全局变量
int board_w = 9, board_h = 6;     //定义棋盘的宽和高
int n_boards = 15, delay = 500 ;    //定义获取图像数目,相机的拍摄延时
double image_sf = 0.5;     //定义缩放比例为0.5
vector<cv::Mat> parameter1, parameter2;     //定义两个相机的参数矩阵
cv::Mat M1, D1, M2, D2;     //定义两个相机的内外参矩阵
const char *imageList = "../get_depth_information2/list.txt";     //定义文件读取列表
bool useUncalibrated = false;     //不使用未校准方法
bool displayCorners = true;     //显示角点
bool showUndistores = true;     //显示校正后的图像
bool isVerticalStereo = false; 
const char* point_cloud_filename = 0;
const char* point_cloud_filename1 = 0;
int sign = 0, sign_1 = 0;
double mid_x, mid_y, mid_z, sita, sita1;

int main()
{
	system("color 3E");
	PlaySound(TEXT("F:\\Computer vision\\Practice\\get_depth_information2\\get_depth_information2\\5885.wav"), NULL, SND_FILENAME | SND_ASYNC);
	if (License_verification() == -1) exit(0);
	parameter1 = Camera_calibration(board_w, board_h, n_boards, delay, image_sf, 0);     //左侧相机单目标定
	M1 = parameter1[0];
	D1 = parameter1[1];
	cout << "\n摄像头1的内在参数为：\n" << M1 << "\n摄像头1的畸变参数为：\n" << D1 << endl;
	system("color 6F");
	parameter2 = Camera_calibration(board_w, board_h, n_boards, delay, image_sf, 1);     //右侧相机单目标定
	M2 = parameter2[0];
	D2 = parameter2[1];
	cout << "\n摄像头2的内在参数为：\n" << D2 << "\n摄像头2的畸变参数为：\n" << D2 << endl;
	system("color BE");
	cv::Mat R, T, E, F;
	cv::Size imageSize;
	result *camera;
	camera = StereoCalib
	(board_w,
		board_h,
		M1,
		D1,
		M2,
		D2,
		imageList,     //图像文件列表 
		useUncalibrated,     //是否使用未校准方法
		displayCorners,     //是否显示角点
		showUndistores,     //是否显示校正后的图像
		isVerticalStereo     //判断图像是垂直还是水平
	);     //双目标定、校正、对应
	//提取矩阵
	R = camera->R;
	T = camera->T;
	E = camera->E;
	F = camera->F;
	imageSize = camera->imageSize;
	cout << "立体校正参数为：\n" << "旋转矩阵：\n" << R << "\n" << "平移矩阵：\n" << T << "\n" << "本征矩阵：\n" << E << "\n" << "基本矩阵：\n" << F << "\n";
	system("color 3E");
	//一切准备就绪，可以开始识别了
	cout << "\n一切准备就绪";
	cv::VideoCapture picture1(0);
	cv::VideoCapture picture2(1);
	cv::Mat obtainl, obtainr;
	int jsq = 0;
	cv::Mat frame1;
	cv::Mat frame2;
	if (!picture1.isOpened())
		return 0;
	if (!picture2.isOpened())
		return 0;
	point_cloud_filename = "point_cloud.txt";//保存云点
	point_cloud_filename1 = "point_cloud1.txt";//保存云点
	for (;;)
	{
		picture1 >> frame1;
		cv::imshow("camera_left", frame1);
		picture2 >> frame2;
		cv::imshow("camera_right", frame2);
		if (((cv::waitKey(10)) & 255) == 109)
		{
			PlaySound(TEXT("F:\\Computer vision\\Practice\\get_depth_information2\\get_depth_information2\\9304.wav"), NULL, SND_FILENAME | SND_SYNC);
			picture1 >> obtainl;
			picture2 >> obtainr;
			cout << "\n检测到牛奶盒" << jsq++ << "\n";
			cv::imshow("捕获（左）", obtainl);
			cv::imshow("捕获（右）", obtainr);
			cv::imwrite("666.jpg", obtainl);
			cv::Mat obtainlr, obtainrr, disp_1, vdisp_1;
			cv::Mat R11, R22, P11, P22, Q1, map11_1, map12_1, map21_1, map22_1;
			stereoRectify(M1, D1, M2, D2, imageSize, R, T, R11, R22, P11, P22, Q1, 0);     //bouguet算法
			isVerticalStereo = fabs(P22.at<double>(1, 3)) > fabs(P22.at<double>(0, 3));     //判断图像是垂直还是水平
			//矫正映射
			initUndistortRectifyMap(M1, D1, R11, P11, imageSize, CV_16SC2, map11_1, map12_1);
			initUndistortRectifyMap(M2, D2, R22, P22, imageSize, CV_16SC2, map21_1, map22_1);
			cv::remap(obtainl, obtainlr, map11_1, map12_1, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
			cv::remap(obtainr, obtainrr, map21_1, map22_1, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
			cv::Ptr<cv::StereoSGBM>stereo = cv::StereoSGBM::create
			(0, 128, 7, 100, 1000, -1, 15, 15, 150, 1, cv::StereoSGBM::MODE_HH);
			stereo->compute(obtainlr, obtainrr, disp_1);
			cv::normalize(disp_1, vdisp_1, 0, 256, cv::NORM_MINMAX, CV_8U);
			cv::imshow("disparity", vdisp_1);
			cv::Mat xyz_1, xyz_2, xyz_3;
			cv::reprojectImageTo3D(vdisp_1, xyz_1, Q1, true);
			saveXYZWrapper(point_cloud_filename, xyz_1);
			int m;
			m = CountLines(point_cloud_filename);
			savexyzwrapper(point_cloud_filename1, xyz_1, m);
			if ((cv::waitKey() & 255) == 97)
			{
				if (sign != 0)
				{
					cv::destroyWindow("3D Viewer");
				}
				sign++;
				int c = pcd_view();
			}
			int *aim;
			cv::Point2d p1, p2, p3, p4;
			aim = findcontours();
			p1.x = aim[0]; p1.y = aim[1];
			p2.x = aim[2]; p2.y = aim[3];
			p3.x = aim[4]; p3.y = aim[5];
			p4.x = aim[6]; p4.y = aim[7];			
			//获取四个点的三维坐标并校正缺失值（最小二乘法插值）
			double *points1, *points2, *points3, *points4;
			points1 = Least_squares_interpolation(xyz_1, p1);
			points2 = Least_squares_interpolation(xyz_1, p2);
			points3 = Least_squares_interpolation(xyz_1, p3);
			points4 = Least_squares_interpolation(xyz_1, p4);
			cout << "检测到最顶端像素位置为：" << points1[0] << "," << points1[1] << " 深度为：" << points1[2] << "\n";
			cout << "检测到最底端像素位置为：" << points2[0] << "," << points2[1] << " 深度为：" << points2[2] << "\n";
			cout << "检测到最左端像素位置为：" << points3[0] << "," << points3[1] << " 深度为：" << points3[2] << "\n";
			cout << "检测到最右端像素位置为：" << points4[0] << "," << points4[1] << " 深度为：" << points4[2] << "\n";
			//位姿估计
			mid_x = (points1[0] + points2[0])*0.5;
			mid_y = (points3[1] + points4[1])*0.5;
			mid_z = (points1[2] + points2[2])*0.5;
			sita = atan2(abs(points3[2] - points4[2]), abs(points3[0] - points4[0]));
			sita1 = sita * 180 / pi;
			double a = sqrt(pow(points1[0] - points3[0], 2) + pow(points1[1] - points3[1], 2) + pow(points1[2] - points3[2], 2));
			double b = sqrt(pow(points2[0] - points3[0], 2) + pow(points2[1] - points3[1], 2) + pow(points2[2] - points3[2], 2));
			if (abs(points1[1] - points2[1]) > abs(points3[1] - points4[1]))
			{
				if (a > b)
					cout << "牛奶盒目前处于正放直立状态，中点坐标为" << mid_x << "，" << mid_y << "，" << mid_z << "角度为：" << sita1 << "\n";
				else
					cout << "牛奶盒目前处于倒放直立状态，中点坐标为" << mid_x << "，" << mid_y << "，" << mid_z << "角度为：" << sita1 << "\n";
			}
			else
			{
				if (a > b)
					cout << "牛奶盒目前处于正放躺倒状态，中点坐标为" << mid_x << "，" << mid_y << "，" << mid_z << "角度为：" << sita1 << "\n";
				else
					cout << "牛奶盒目前处于倒放躺倒状态，中点坐标为" << mid_x << "，" << mid_y << "，" << mid_z << "角度为：" << sita1 << "\n";
			}
			//关闭窗口
			cv::destroyWindow("捕获（左）");
			cv::destroyWindow("捕获（右）");
			cv::destroyWindow("disparity");
			//按Esc退出
			if (((cv::waitKey(10)) & 255) == 27)
				break;
		}	
	}
	return 0;
}