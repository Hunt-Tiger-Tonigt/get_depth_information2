#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include<opencv2/opencv.hpp>
#include<iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

LINETYPE *camera = NULL;

result *StereoCalib
(   int board_w,     //���̵Ŀ��
	int board_h,     //���̵ĸ߶�
	cv::Mat M1, 
	cv::Mat D1, 
	cv::Mat M2, 
	cv::Mat D2,
	const char *imageList,     //ͼ���ļ��б�
	bool useUncalibrated,     //�Ƿ�ʹ��δУ׼����
	bool displayCorners,     //�Ƿ���ʾ�ǵ�
	bool showUndistores,     //�Ƿ���ʾУ�����ͼ��
	bool isVerticalStereo     //�ж�ͼ���Ǵ�ֱ����ˮƽ
)

{
	const char* point_cloud_filename = 0;
	const char* point_cloud_filename1 = 0;
	const int maxScale = 1;
	const float squareSize = 1.f;
	FILE* f = fopen(imageList, "rt");     //����fΪ��ͼ���б�ģʽΪֻ��
	int N = board_w * board_h;     //����NΪ���̽ǵ����
	cv::Size board_sz = cv::Size(board_w, board_h);     //����board_szΪsize������
	int jsq = 0;
	int i, j, lr;
	vector<string> imageNames[2];
	vector<cv::Point3f> boardModel;
	vector<vector<cv::Point3f>> objectPoints;
	vector<vector<cv::Point2f>> points[2];
	vector<cv::Point2f> corners[2];
	bool found[2] = { false,false };
	cv::Size imageSize;

	//��ȡ�������̵�ͼƬ����
	if (!f)
	{
		cout << "�򲻿��ļ�" << imageList << endl;     //Ҫ�Ǵ򲻿������Ҳ����ļ�����gg
		return NULL;
	}

	//�����̽ǵ��������boardmodel�У����о��루��ȣ���ϢΪ0
	for (i = 0; i < board_h; i++)
		for (j = 0; j < board_w; j++)
			boardModel.push_back(
				cv::Point3f((float)(i*squareSize), (float)(j*squareSize), 0.f));
	i = 0;
	for (;;)
	{
		char buf[1024];    //����һ������Ϊ1024���ֽڵĿռ䣬��Ϊ�ַ�����ʹ��
		lr = i % 2;     //lr�����ж���ȡͼ��Ϊ�����������ұ����
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
			cv::drawChessboardCorners(cimg, cv::Size(board_w, board_h), corners[lr], found[lr]);     //�������̽ǵ�
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
	system("color 67");

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
			/*
			cameraMatrix1�C ��һ���������.
			distCoeffs1�C ��һ������������.
			cameraMatrix2�C �ڶ����������.
			distCoeffs2�C �ڶ�������������.
			imageSize�C ����У����ͼ���С.
			R�C ��һ�͵ڶ��������ϵ֮�����ת����
			T�C ��һ�͵ڶ��������ϵ֮���ƽ�ƾ���.
			R1�C �����һ�������3x3�����任(��ת����) .
			R2�C ����ڶ��������3x3�����任(��ת����) .
			P1�C�ڵ�һ̨������µ�����ϵͳ(��������)��� 3x4 ��ͶӰ����
			P2�C�ڵڶ�̨������µ�����ϵͳ(��������)��� 3x4 ��ͶӰ����
			Q�C�������Ӳ�ӳ���
			*/
			isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));     //�ж�ͼ���Ǵ�ֱ����ˮƽ
			//����ӳ��
			initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);
			/*
			cameraMatrix���������������ڲ�������
			distCoeffs������������������ϵ������
			R��������ĵ�һ�͵ڶ��������ϵ֮�����ת����
			newCameraMatrix���������У�����3X3���������
			size����������ɼ�����ʧ��ͼ��ߴ�
			m1type����map1���������ͣ�������CV_32FC1��CV_16SC2
			map1���������X������ӳ�����
			map2���������Y������ӳ�����
			*/
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
			pair.create(imageSize.height, imageSize.width * 2, CV_8UC3);    //ͼ��ˮƽ
		else
			pair.create(imageSize.height * 2, imageSize.width, CV_8UC3);    //ͼ��ֱ

																			//���ж�Ӧ
		cv::Ptr<cv::StereoSGBM>stereo = cv::StereoSGBM::create
		(0, 128, 7, 100, 1000, -1, 15, 15, 150, 1, cv::StereoSGBM::MODE_HH);

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
			for (int i = 0; i < vdisp.rows; i++)
			{
				for (int j = 0; j < vdisp.cols; j++)
				{
					o_file << int(vdisp.at<uchar>(cv::Point(j, i))) << "   ";
				}
				o_file << "\n";
			}
			point_cloud_filename = "point_cloud.txt";//�����Ƶ�
			point_cloud_filename1 = "point_cloud1.txt";//�����Ƶ�
			if (true)
			{
				printf("storing the point cloud...");
				fflush(stdout);
				cv::Mat xyz;
				cv::reprojectImageTo3D(vdisp, xyz, Q, true);
				cv::Point2d p;
				p.x = 320; p.y = 240;
				cout << "���ĵ�����Ϊ��" << xyz.at<cv::Vec3f>(p) << "\n";
				saveXYZWrapper(point_cloud_filename, xyz);
				int m;
				m = CountLines(point_cloud_filename);
				savexyzwrapper(point_cloud_filename1, xyz,m);
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
			if ((cv::waitKey() & 255) == 97)
			{
				cv::destroyWindow("3D Viewer"); 
				int c=pcd_view();
			}
			if ((cv::waitKey() & 255) == 27)
				break;
		}
	}
	cv::destroyWindow("��Ӧͼ��");
	cv::destroyWindow("Corners");
	result *Stereo = new result;
	Stereo->R = R;
	Stereo->T = T;
	Stereo->E = E;
	Stereo->F = F;
	Stereo->imageSize = imageSize;
	return Stereo;
}
