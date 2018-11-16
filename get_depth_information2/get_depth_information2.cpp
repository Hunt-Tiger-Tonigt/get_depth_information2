/*******************************
*  @Function   get_depth_information
*  @Works      ����̨������е�Ŀ�궨������������궨��У������Ӧ�����ɾ��������Ϣ��ͼ��
*  @Author     Hunt Tiger Tonight
*  @Platform   VS2015 C++
*  @Connect    phone��18398621916/QQ:136768916
*  @Date       2018-11-06
********************************/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <string>
#include <windows.h>
#include <cstring>
#include <shellapi.h>
#include <tchar.h>
#include <fstream> 
#pragma comment(lib, "shell32.lib")

using namespace std;

//�����������
static void saveXYZ(const char* filename, const cv::Mat& mat)
{
	const double max_z = 1.0e4;
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
	}
	fclose(fp);
}

vector<cv::Mat> Camera_calibration(
	int board_w,     //���̵Ŀ��
	int board_h,     //���̵ĸ߶�
	int n_boards,     //���궨ͼ�����Ŀ��������������������ȡ��Ϊ�˱�֤��������⾫�ȣ�����������Ҫ10�����ϵ�ͼ��      
	int delay,     //�����������ʱΪ1s
	double image_sf,     //���ű���Ϊ0.5
	int cap     //ѡ��������
)
{
	int n = 0;
	int i = 0;
	char filename[1024];
	cv::Mat img_grayl;
	cv::Mat img_grayr;
	vector<cv::Mat> parameter;
	int board_n = board_w * board_h;
	cv::Size board_sz = cv::Size(board_w, board_h);     //board_sz����Ϊsize���͵�����

														//������ͷ
	cv::VideoCapture capture(cap);
	cv::VideoCapture capture1(1);
	if (!capture.isOpened())
	{
		cout << "\n�޷�������ͷ��";
		return parameter;
	}
	//���䴢����
	vector<vector<cv::Point2f>> image_points;     //����������ͼ��ǵ��������������е�������
	vector<vector<cv::Point3f>> object_points;     //������������ϵ�нǵ��������������е�������

												   //�����������ͼ��ֱ���ҵ����̣����������е�ͼ��
	double last_captured_timestamp = 0;     //��ʼ�����һ�β���ͼ��ʱ��Ϊ0
	cv::Size image_size;     //����size�ͺ���

							 //��ʼ������ֱ���ҵ�ȫ��ͼ��
	while (image_points.size() < (size_t)n_boards)
	{
		cv::Mat image0, image, image1;     //����ԭʼͼ������Լ����ͼ�����
		capture >> image0;     //��ԭʼͼ��浽capture��
		image_size = image0.size();     //��ȡimage0�Ĵ�С
		cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);     //����ͼ�񣬺����������P268

																						 //Ѱ������
		vector<cv::Point2f> corners;     //����ǵ��������
		bool found = cv::findChessboardCorners(image, board_sz, corners);     //Ѱ�ҽǵ㺯�������p568
																			  //��������
		drawChessboardCorners(image, board_sz, corners, found);     //���ƽǵ㣬���p569

																	//����ҵ������ˣ��Ͱ�������������
		double timestamp = (double)clock() / CLOCKS_PER_SEC;     //��ȡʱ���

		if (found && timestamp - last_captured_timestamp > 1)     //���Ѱ�ҵ�������
		{
			if (cap == 0)
			{
				n++;
				sprintf_s(filename, "left%.2d.jpg", n);
				cv::cvtColor(image0, img_grayl, cv::COLOR_BGR2GRAY);
				cv::imwrite(filename, img_grayl);
				cout << "\n������ " << filename << "�ļ�����Ŀ¼��" << endl;
				capture1 >> image1;
				sprintf_s(filename, "right%.2d.jpg", n);
				cv::cvtColor(image1, img_grayr, cv::COLOR_BGR2GRAY);
				cv::imwrite(filename, img_grayr);
				cout << "������ " << filename << "�ļ�����Ŀ¼��" << endl;
			}
			last_captured_timestamp = timestamp;     //����ǰʱ�����Ϊ���һ��ʱ��
			image ^= cv::Scalar::all(255);     //��ͼ�����һ��������㣬255Ϊ��ɫ�������ڱ�ף��ױ��
			cv::Mat mcorners(corners);     //���ƾ��󣨱����ƻ�ԭ�о���
			mcorners *= (1.0 / image_sf);     //���Ž�����
			image_points.push_back(corners);     //��image_points�����corners������ע��һ�£��˾��൱�ڣ���imageͼ���ϵ�����һ������ͼ��
			object_points.push_back(vector<cv::Point3f>());     //��object_points�����Point3f���ͺ�����ͬ���ȼ���һ����û����⵽��ͼ����һ���վ����ʾ
																//���������ʵ�Ҿ����ҵ�����е����⣬�ҵ�����ǣ������ͼ����ռ�ڴ��С���������ţ���ȡͼ��ֱ����Ŀ�ﵽԤ��ֵ
			vector<cv::Point3f> & opts = object_points.back();     //opts����Options,����˵���ǽ����ͼ������һλ��С����
			opts.resize(board_n);     //����������С
			for (int j = 0; j < board_n; j++)
			{
				opts[j] = cv::Point3f(static_cast<float>(j / board_w), static_cast<float>(j % board_w), 0.0f);     //����ά���ݴ���opts��,ע�⣬����ط������ǿ��ת������Ȼ���������������
			}
			cout << "\n���ռ���" << static_cast<uint>(image_points.size()) << "������ͼ���ܹ���Ҫ" << n_boards << "������ͼ��\n" << endl;
		}
		cv::imshow("Calibration", image);     //��ʾͼ��

											  //�ȴ�ʱ��Ϊ30ms����������ʱ�����, �û�����ESC(ASCII��Ϊ27),������ѭ��,����,������ѭ��
		if (((cv::waitKey(30)) & 255) == 27)
			return parameter;

	}
	//����ѭ��
	cv::destroyWindow("Calibration");     //���ٴ���
	cout << "\n\n���ڽ������...\n" << endl;

	//У׼���
	cv::Mat intrinsic_matrix, distortion_coeffs;     //instrinsic_matrix:�������ڲ�����3*3���� distortion_coeffs������ϵ����k1��k2��p1��p2
	double err = cv::calibrateCamera(
		object_points,
		image_points,
		image_size,
		intrinsic_matrix,
		distortion_coeffs,
		cv::noArray(),
		cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT

	);     //У׼������������P582

	cout << "***Done!\n\nReprojection error is " << err;
	//�����޻��������ת��ӳ��
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(
		intrinsic_matrix,
		distortion_coeffs,
		cv::Mat(),
		intrinsic_matrix,
		image_size,
		CV_16SC2,
		map1,
		map2
	);     //�����޻��������ת��ӳ�䣬���P590
		   //��ʾ�����ĺ��ͼ��
	int secs = 5;
	clock_t delay1 = secs * CLOCKS_PER_SEC;
	clock_t start = clock();
	while (clock() - start<delay1)
	{
		cv::Mat image, image0;
		capture >> image0;
		if (image0.empty())
			break;
		cv::remap(
			image0,
			image,
			map1,
			map2,
			cv::INTER_LINEAR,
			cv::BORDER_CONSTANT,
			cv::Scalar()
		);     //����remap���´���ͼ��
		cv::imshow("Undistored", image);
		if (((cv::waitKey(30)) & 255) == 27)
			break;
	}
	//cv::waitKey(3000);
	cv::destroyWindow("Undistored");
	parameter.push_back(intrinsic_matrix);
	parameter.push_back(distortion_coeffs);
	return parameter;
}

//���庯���������������������ͼ�����кŵ��ļ������̵ĺ�����������̵����������
//��Ŀ�Ƿ���У׼�������жϲ���Hartlely������Bouguet��������ѡ��Bouguet��
static void StereoCalib(
	const char *imageList,
	int nx,
	int ny,
	bool useUncalibrated,
	cv::Mat M1,
	cv::Mat M2,
	cv::Mat D1,
	cv::Mat D2)
{
	//����һЩ��
	bool displayCorners = true;
	bool showUndistores = true;
	bool isVerticalStereo = false;
	const char* point_cloud_filename = 0;
	const int maxScale = 1;
	const float squareSize = 1.f;
	FILE* f = fopen(imageList, "rt");     //����fΪ��ͼ���б�ģʽΪֻ��
	int i, j, lr;
	int N = nx * ny;     //����NΪ���̽ǵ����
	cv::Size board_sz = cv::Size(nx, ny);     //����board_szΪsize������
	vector<string> imageNames[2];
	vector<cv::Point3f> boardModel;
	vector<vector<cv::Point3f>> objectPoints;
	vector<vector<cv::Point2f>> points[2];
	vector<cv::Point2f> corners[2];
	bool found[2] = { false,false };
	cv::Size imageSize;
	int ddeph = -1;

	//��ȡ�������̵�ͼƬ����
	if (!f)
	{
		cout << "�򲻿��ļ�" << imageList << endl;     //Ҫ�Ǵ򲻿������Ҳ����ļ�����gg
		return;
	}

	//�����̽ǵ��������boardmodel�У����о��루��ȣ���ϢΪ0
	for (i = 0; i < ny; i++)
		for (j = 0; j < nx; j++)
			boardModel.push_back(
				cv::Point3f((float)(i*squareSize), (float)(j*squareSize), 0.f));
	i = 0;
	for (;;)
	{
		char buf[1024];    //����һ������Ϊ1024���ֽڵĿռ䣬��Ϊ�ַ�����ʹ��
		lr = i % 2;     //lr�����ж���ȡͼ��Ϊ�����������ұ����
		cout << "\nlr=" << lr << endl;
		if (lr == 0)
			found[0] = found[1] = false;     //���lrΪ���ж�Ϊ������
		cout << "\nfound=" << found[0] << found[1] << endl;
		if (!fgets(buf, sizeof(buf) - 3, f))
			break;     //���û�ж�������ֹѭ��
		size_t len = strlen(buf);     //��ȡbuf���ַ������ȣ�����len��
		while (len > 0 && isspace(buf[len - 1]))     //isspace���ж��Ƿ�Ϊ�ո��Ʊ�������򷵻ط���ֵ������ʵ����Ҳ�����⣩
			buf[--len] = '\0';
		if (buf[0] == '#')     //����#�����
			continue;
		cv::Mat img = cv::imread(buf, 0);     //��ȡbuf�����ַ��ָ�����Ƭ������img��
		if (img.empty())     //Ҫ�Ƕ����ˣ�����ֹѭ��
			break;
		imageSize = img.size();     //��img�Ĵ�С����imagesize��
		imageNames[lr].push_back(buf);     //��buf����imagename����
		i++;

		//�����������û���ҵ����̣���Ҳû��Ҫ��������ȥ����
		if (lr == 1 && !found[0])
			continue;
		//������
		int s;
		for (s = 1; s <= maxScale; s++)
		{
			cv::Mat timg = img;
			if (s > 1)
				resize(img, timg, cv::Size(), s, s, cv::INTER_CUBIC);     //����ͼ��
			found[lr] = cv::findChessboardCorners(timg, board_sz, corners[lr]);     //����find�������صľ��ǲ���ֵ��ֱ�ӷ��ص�found���ж���û�ҵ�
			if (found[lr] || s == maxScale)
			{
				cv::Mat mcorners(corners[lr]);  //���ǵ��λ���������浽mcorner�У����ƻ�
				mcorners *= (1. / s);     //���ǵ�λ�û�ԭΪδ���ŵ�λ��
			}
			if (found[lr])
				break;
		}
		if (displayCorners)
		{
			cout << buf << endl;
			cv::Mat cimg;
			cv::cvtColor(img, cimg, cv::COLOR_GRAY2BGR);     //��img�е�ͼ��ת��ΪRGBͼ��
			cv::drawChessboardCorners(cimg, cv::Size(nx, ny), corners[lr], found[lr]);     //�������̽ǵ�
			cv::imshow("Corners", cimg);     //��ʾͼ��cimg����������ΪCorners
			if ((cv::waitKey(0) & 255) == 27)    //�������esc�˳�
				exit(-1);
		}
		else
			cout << '.';
		if (lr == 1 && found[0] && found[1])
		{
			objectPoints.push_back(boardModel);     //�����̵�����λ����Ϣ����objectpoint��
			points[0].push_back(corners[0]);     //�����ǵ�λ�þ������points[0]��
			points[1].push_back(corners[1]);     //���Ҳ�ǵ�λ�þ������points[1]��
		}
	}
	fclose(f);     //�ر��ļ�

				   //����У��
	cv::Mat  R, T, E, F;
	cout << "\n���ڽ����������У��";
	cv::stereoCalibrate(
		objectPoints,     //objectPoints���洢�궨�ǵ�����������ϵ�е�λ��
		points[0],     //imagePoints1���洢�궨�ǵ��ڵ�һ��������µ�ͶӰ�������������
		points[1],     //imagePoints2���洢�궨�ǵ��ڵڶ���������µ�ͶӰ�������������
		M1,     //cameraMatrix1������/����͵ĵ�һ����������������
				//ע�⣺���CV_CALIB_USE_INTRINSIC_GUESS , CV_CALIB_FIX_ASPECT_RATIO ,CV_CALIB_FIX_INTRINSIC , or CV_CALIB_FIX_FOCAL_LENGTH���е�һ��������־�����ã�������������һЩ��ȫ��������Ҫ����ʼ��
		D1,     //distCoeffs1����һ�������������/����ͻ�������
		M2,     //cameraMatrix2������/����͵ĵ�һ����������������
		D2,     //distCoeffs2����һ�������������/����ͻ�������
		imageSize,     //imageSize��ͼ��Ĵ�С
		R,     //R������ͣ���һ�͵ڶ��������֮�����ת����
		T,     //T������ͣ���һ�͵ڶ��������֮���ƽ�ƾ���
		E,     //E������ͣ���������
		F,     //F������ͣ���������
		cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH,
		/*
		CV_CALIB_FIX_INTRINSIC ����ñ�־�����ã���ô�ͻ�̶������cameraMatrix��distCoeffs���䣬ֻ���R, T, E, F
		CV_CALIB_USE_INTRINSIC_GUESS �����û��ṩ��cameraMatrix��distCoeffsΪ��ʼֵ��ʼ����
		CV_CALIB_FIX_PRINCIPAL_POINT ���������в���ı������λ��
		CV_CALIB_FIX_FOCAL_LENGTH ���������в���ı佹��
		CV_CALIB_FIX_ASPECT_RATIO �̶�fx/fy�ı�ֵ��ֻ��fy��Ϊ�ɱ����������Ż����㡣
		����CV_CALIB_USE_INTRINSIC_GUESSû�б����ã�fx��fy���ᱻ���ԡ�ֻ��fx/fy�ı�ֵ�ڼ����лᱻ�õ���)
		CV_CALIB_SAME_FOCAL_LENGTH ǿ�Ʊ�������������Ľ�����ͬ
		CV_CALIB_ZERO_TANGENT_DIST ������䱣��Ϊ��
		CV_CALIB_FIX_K1, ..., CV_CALIB_FIX_K6 ���������в��ı���Ӧ��ֵ����������� CV_CALIB_USE_INTRINSIC_GUESS ����ʹ���û��ṩ�ĳ�ʼֵ����������Ϊ��
		CV_CALIB_RATIONAL_MODEL ����ģ�͵�ѡ����������˸ò���������ʹ�ø���ȷ�Ļ���ģ�ͣ�distCoeffs�ĳ��Ⱦͻ���8
		*/
		cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5)
		/*TermCriteriaģ���࣬��Ϊ�����㷨����ֹ������
		���������Ҫ3��������һ�������ͣ��ڶ�������Ϊ�����������������һ�����ض�����ֵ��
		������TermCriteria::COUNT��TermCriteria::EPS��cv::TermCriteria::COUNT|cv::TermCriteria::EPS��
		�ֱ�����ŵ�����ֹ����Ϊ�ﵽ������������ֹ����������ֵ��ֹ���������߶���Ϊ������ֹ����
		������õ����֣�����������Ϊ100����ֵΪ10^-5
		*/
	);
	cout << "\n�㶨��������������ͼ�񣬰�ESC�˳�\n\n";

	//У�����
	vector<cv::Point3f> lines[2];
	double avgErr = 0;
	int nframes = (int)objectPoints.size();
	for (i = 0; i < nframes; i++)
	{
		vector<cv::Point2f>&pt0 = points[0][i];
		vector<cv::Point2f>&pt1 = points[1][i];
		cv::undistortPoints(pt0, pt0, M1, D1, cv::Mat(), M1);    
		//���ݹ۲쵽�ĵ�����������������
		/*
		src�� �۲쵽�ĵ�����
		dst���ڷ�ʧ��ͷ���͸�ӱ任�������������ꡣ
		cameraMatrix
		distCoeffs
		R - ����ռ��е������任��3x3���󣩣��������Ϊ�գ���ʹ�����ת����
		P - �µ��������3x3�����µ�ͶӰ����3x4����
		*/
		cv::undistortPoints(pt1, pt1, M2, D2, cv::Mat(), M2);
		cv::computeCorrespondEpilines(pt0, 1, F, lines[0]);
		cv::computeCorrespondEpilines(pt1, 2, F, lines[1]);     //���߼���

		for (j = 0; j < N; j++)
		{
			double err = fabs(pt0[j].x*lines[1][j].x + pt0[j].y*lines[1][j].y + lines[1][j].z) +
				fabs(pt1[j].x*lines[0][j].x + pt1[j].y*lines[0][j].y + lines[0][j].z);
			avgErr += err;
		}
	}
	cout << "ƽ�����Ϊ��" << avgErr / (nframes*N) << endl;

	//���㡢��ʾУ׼֮���ͼ��
	if (showUndistores)
	{
		cv::Mat R1, R2, P1, P2, map11, map12, map21, map22, Q;
		//�����Ŀ��У׼�������bouguet��
		if (!useUncalibrated)
		{
			stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q, 0);     //bouguet�㷨
			isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));     //�ж�ͼ���Ǵ�ֱ����ˮƽ
																						  //����ӳ��
			initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);
			initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map21, map22);
		}
		else
		{
			vector<cv::Point2f> allpoints[2];
			for (i = 0; i < nframes; i++)
			{
				copy(points[0][i].begin(), points[0][i].end(),
					back_inserter(allpoints[0]));
				copy(points[1][i].begin(), points[1][i].end(),
					back_inserter(allpoints[1]));
			}
			cv::Mat F = findFundamentalMat(allpoints[0], allpoints[1], cv::FM_8POINT);
			cv::Mat H1, H2;
			cv::stereoRectifyUncalibrated(allpoints[0], allpoints[1], F, imageSize,
				H1, H2, 3);
			R1 = M1.inv() * H1 * M1;
			R2 = M2.inv() * H2 * M2;

			cv::initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);
			cv::initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map21, map22);
		}

		//У������ʾͼ��
		cv::Mat pair;
		if (!isVerticalStereo)
			pair.create(imageSize.height, imageSize.width * 2, CV_8UC3);
		else
			pair.create(imageSize.height * 2, imageSize.width, CV_8UC3);

		//���ж�Ӧ
		cv::Ptr<cv::StereoSGBM>stereo = cv::StereoSGBM::create
		(-64, 128, 11, 100, 1000, 32, 0, 15, 1000, 16, cv::StereoSGBM::MODE_HH);

		for (i = 0; i < nframes; i++)
		{
			cv::Mat img1 = cv::imread(imageNames[0][i].c_str(), 0);
			cv::Mat img2 = cv::imread(imageNames[1][i].c_str(), 0);
			cv::Mat img1r, img2r, disp, vdisp;
			if (img1.empty() || img2.empty())
				continue;
			cv::remap(img1, img1r, map11, map12, cv::INTER_LINEAR);
			cv::remap(img2, img2r, map21, map22, cv::INTER_LINEAR);

			if (!isVerticalStereo || !useUncalibrated)
			{
				stereo->compute(img1r, img2r, disp);
				cv::normalize(disp, vdisp, 0, 256, cv::NORM_MINMAX, CV_8U);
				cv::imshow("disparity", vdisp);
			}

			char* rute = "dispdata.txt";
			ofstream o_file(rute); //����ļ�����������������ļ�  
			for (int i = 0; i<vdisp.rows; i++)
			{
				for (int j = 0; j<vdisp.cols; j++)
				{
					o_file << int(vdisp.at<uchar>(cv::Point(j, i))) << "   ";
				}
				o_file << "\n";
			}
			point_cloud_filename = "point_cloud.txt";//�����Ƶ�
			if (point_cloud_filename)
			{
				printf("storing the point cloud...");
				fflush(stdout);
				cv::Mat xyz;
				cv::reprojectImageTo3D(vdisp, xyz, Q, true);
				saveXYZ(point_cloud_filename, xyz);
				printf("\n");
			}

			if (!isVerticalStereo)  //ˮƽ��Ӧ��ֱ��Ӧ
			{
				cv::Mat part = pair.colRange(0, imageSize.width);    //��ȡpair�е�һ�з���part��
				cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
				part = pair.colRange(imageSize.width, imageSize.width * 2);
				cvtColor(img2r, part, cv::COLOR_GRAY2BGR);
				for (j = 0; j < imageSize.height; j += 16)
					cv::line(pair, cv::Point(0, j), cv::Point(imageSize.width * 2, j), cv::Scalar(0, 255, 0));
			}
			else
			{
				cv::Mat part = pair.rowRange(0, imageSize.height);
				cv::cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
				part = pair.rowRange(imageSize.height, imageSize.height * 2);
				cv::cvtColor(img2r, part, cv::COLOR_GRAY2BGR);
				for (j = 0; j < imageSize.width; j += 16)
					line(pair, cv::Point(j, 0), cv::Point(j, imageSize.height * 2),
						cv::Scalar(0, 255, 0));
			}
			cv::imshow("��Ӧͼ��", pair);
			if ((cv::waitKey() & 255) == 27)
				break;
		}
	}
}

int main()
{
	int board_w = 9, board_h = 6;
	vector<cv::Mat> parameter1, parameter2;
	cv::Mat intrinsic_matrix1, distortion_coeffs1, intrinsic_matrix2, distortion_coeffs2;
	parameter1 = Camera_calibration(9, 6, 14, 500, 0.5, 0);
	intrinsic_matrix1 = parameter1[0];
	distortion_coeffs1 = parameter1[1];
	cout << "\n����ͷ1�����ڲ���Ϊ��" << intrinsic_matrix1 << "\n����ͷ1�Ļ������Ϊ��" << distortion_coeffs1 << endl;
	parameter2 = Camera_calibration(9, 6, 14, 500, 0.5, 1);
	intrinsic_matrix2 = parameter2[0];
	distortion_coeffs2 = parameter2[1];
	cout << "\n����ͷ2�����ڲ���Ϊ��" << intrinsic_matrix2 << "\n����ͷ2�Ļ������Ϊ��" << distortion_coeffs2 << endl;
	const char *board_list = "../get_depth_information2/list.txt";
	StereoCalib(board_list, board_w, board_h, false, intrinsic_matrix1, intrinsic_matrix2, distortion_coeffs1, distortion_coeffs2);
	return 0;
}