/*******************************
*  @Function   get_depth_information
*  @Works      ��ȡĿ��ͼ��λ��
*  @Author     Hunt Tiger Tonight
*  @Platform   VS2015 C++
*  @Connect    phone��18398621916/QQ:136768916
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

//ȫ�ֱ���
int board_w = 9, board_h = 6;     //�������̵Ŀ�͸�
int n_boards = 15, delay = 500 ;    //�����ȡͼ����Ŀ,�����������ʱ
double image_sf = 0.5;     //�������ű���Ϊ0.5
vector<cv::Mat> parameter1, parameter2;     //������������Ĳ�������
cv::Mat M1, D1, M2, D2;     //�����������������ξ���
const char *imageList = "../get_depth_information2/list.txt";     //�����ļ���ȡ�б�
bool useUncalibrated = false;     //��ʹ��δУ׼����
bool displayCorners = true;     //��ʾ�ǵ�
bool showUndistores = true;     //��ʾУ�����ͼ��
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
	parameter1 = Camera_calibration(board_w, board_h, n_boards, delay, image_sf, 0);     //��������Ŀ�궨
	M1 = parameter1[0];
	D1 = parameter1[1];
	cout << "\n����ͷ1�����ڲ���Ϊ��\n" << M1 << "\n����ͷ1�Ļ������Ϊ��\n" << D1 << endl;
	system("color 6F");
	parameter2 = Camera_calibration(board_w, board_h, n_boards, delay, image_sf, 1);     //�Ҳ������Ŀ�궨
	M2 = parameter2[0];
	D2 = parameter2[1];
	cout << "\n����ͷ2�����ڲ���Ϊ��\n" << D2 << "\n����ͷ2�Ļ������Ϊ��\n" << D2 << endl;
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
		imageList,     //ͼ���ļ��б� 
		useUncalibrated,     //�Ƿ�ʹ��δУ׼����
		displayCorners,     //�Ƿ���ʾ�ǵ�
		showUndistores,     //�Ƿ���ʾУ�����ͼ��
		isVerticalStereo     //�ж�ͼ���Ǵ�ֱ����ˮƽ
	);     //˫Ŀ�궨��У������Ӧ
	//��ȡ����
	R = camera->R;
	T = camera->T;
	E = camera->E;
	F = camera->F;
	imageSize = camera->imageSize;
	cout << "����У������Ϊ��\n" << "��ת����\n" << R << "\n" << "ƽ�ƾ���\n" << T << "\n" << "��������\n" << E << "\n" << "��������\n" << F << "\n";
	system("color 3E");
	//һ��׼�����������Կ�ʼʶ����
	cout << "\nһ��׼������";
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
	point_cloud_filename = "point_cloud.txt";//�����Ƶ�
	point_cloud_filename1 = "point_cloud1.txt";//�����Ƶ�
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
			cout << "\n��⵽ţ�̺�" << jsq++ << "\n";
			cv::imshow("������", obtainl);
			cv::imshow("�����ң�", obtainr);
			cv::imwrite("666.jpg", obtainl);
			cv::Mat obtainlr, obtainrr, disp_1, vdisp_1;
			cv::Mat R11, R22, P11, P22, Q1, map11_1, map12_1, map21_1, map22_1;
			stereoRectify(M1, D1, M2, D2, imageSize, R, T, R11, R22, P11, P22, Q1, 0);     //bouguet�㷨
			isVerticalStereo = fabs(P22.at<double>(1, 3)) > fabs(P22.at<double>(0, 3));     //�ж�ͼ���Ǵ�ֱ����ˮƽ
			//����ӳ��
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
			//��ȡ�ĸ������ά���겢У��ȱʧֵ����С���˷���ֵ��
			double *points1, *points2, *points3, *points4;
			points1 = Least_squares_interpolation(xyz_1, p1);
			points2 = Least_squares_interpolation(xyz_1, p2);
			points3 = Least_squares_interpolation(xyz_1, p3);
			points4 = Least_squares_interpolation(xyz_1, p4);
			cout << "��⵽�������λ��Ϊ��" << points1[0] << "," << points1[1] << " ���Ϊ��" << points1[2] << "\n";
			cout << "��⵽��׶�����λ��Ϊ��" << points2[0] << "," << points2[1] << " ���Ϊ��" << points2[2] << "\n";
			cout << "��⵽���������λ��Ϊ��" << points3[0] << "," << points3[1] << " ���Ϊ��" << points3[2] << "\n";
			cout << "��⵽���Ҷ�����λ��Ϊ��" << points4[0] << "," << points4[1] << " ���Ϊ��" << points4[2] << "\n";
			//λ�˹���
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
					cout << "ţ�̺�Ŀǰ��������ֱ��״̬���е�����Ϊ" << mid_x << "��" << mid_y << "��" << mid_z << "�Ƕ�Ϊ��" << sita1 << "\n";
				else
					cout << "ţ�̺�Ŀǰ���ڵ���ֱ��״̬���е�����Ϊ" << mid_x << "��" << mid_y << "��" << mid_z << "�Ƕ�Ϊ��" << sita1 << "\n";
			}
			else
			{
				if (a > b)
					cout << "ţ�̺�Ŀǰ���������ɵ�״̬���е�����Ϊ" << mid_x << "��" << mid_y << "��" << mid_z << "�Ƕ�Ϊ��" << sita1 << "\n";
				else
					cout << "ţ�̺�Ŀǰ���ڵ����ɵ�״̬���е�����Ϊ" << mid_x << "��" << mid_y << "��" << mid_z << "�Ƕ�Ϊ��" << sita1 << "\n";
			}
			//�رմ���
			cv::destroyWindow("������");
			cv::destroyWindow("�����ң�");
			cv::destroyWindow("disparity");
			//��Esc�˳�
			if (((cv::waitKey(10)) & 255) == 27)
				break;
		}	
	}
	return 0;
}