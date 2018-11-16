/*******************************
*  @Function   get_depth_information
*  @Works      对两台相机进行单目标定，随后进行立体标定、校正、对应，生成具有深度信息的图像
*  @Author     Hunt Tiger Tonight
*  @Platform   VS2015 C++
*  @Connect    phone：18398621916/QQ:136768916
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

//保存点云数据
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
	int board_w,     //棋盘的宽度
	int board_h,     //棋盘的高度
	int n_boards,     //监测标定图像的数目，后面在输入参数里面获取，为了保证参数的求解精度，我们至少需要10张以上的图像      
	int delay,     //相机的拍摄延时为1s
	double image_sf,     //缩放比例为0.5
	int cap     //选择调用相机
)
{
	int n = 0;
	int i = 0;
	char filename[1024];
	cv::Mat img_grayl;
	cv::Mat img_grayr;
	vector<cv::Mat> parameter;
	int board_n = board_w * board_h;
	cv::Size board_sz = cv::Size(board_w, board_h);     //board_sz定义为size类型的数据

														//打开摄像头
	cv::VideoCapture capture(cap);
	cv::VideoCapture capture1(1);
	if (!capture.isOpened())
	{
		cout << "\n无法打开摄像头。";
		return parameter;
	}
	//分配储存面
	vector<vector<cv::Point2f>> image_points;     //定义棋盘中图像角点的输出矩阵（向量中的向量）
	vector<vector<cv::Point3f>> object_points;     //定义物理坐标系中角点的输出矩阵（向量中的向量）

												   //相机不断拍摄图像，直到找到棋盘，并找齐所有的图像。
	double last_captured_timestamp = 0;     //初始化最后一次捕获图像时间为0
	cv::Size image_size;     //构造size型函数

							 //开始搜索，直到找到全部图像
	while (image_points.size() < (size_t)n_boards)
	{
		cv::Mat image0, image, image1;     //构造原始图像矩阵以及输出图像矩阵
		capture >> image0;     //将原始图像存到capture中
		image_size = image0.size();     //获取image0的大小
		cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);     //缩放图像，函数解析详见P268

																						 //寻找棋盘
		vector<cv::Point2f> corners;     //定义角点输出矩阵
		bool found = cv::findChessboardCorners(image, board_sz, corners);     //寻找角点函数，详见p568
																			  //绘制棋盘
		drawChessboardCorners(image, board_sz, corners, found);     //绘制角点，详见p569

																	//如果找到棋盘了，就把他存入数据中
		double timestamp = (double)clock() / CLOCKS_PER_SEC;     //获取时间戳

		if (found && timestamp - last_captured_timestamp > 1)     //如果寻找到了棋盘
		{
			if (cap == 0)
			{
				n++;
				sprintf_s(filename, "left%.2d.jpg", n);
				cv::cvtColor(image0, img_grayl, cv::COLOR_BGR2GRAY);
				cv::imwrite(filename, img_grayl);
				cout << "\n保存了 " << filename << "文件到根目录下" << endl;
				capture1 >> image1;
				sprintf_s(filename, "right%.2d.jpg", n);
				cv::cvtColor(image1, img_grayr, cv::COLOR_BGR2GRAY);
				cv::imwrite(filename, img_grayr);
				cout << "保存了 " << filename << "文件到根目录下" << endl;
			}
			last_captured_timestamp = timestamp;     //将当前时间更新为最后一次时间
			image ^= cv::Scalar::all(255);     //将图像进行一次异或运算，255为白色，即：黑变白，白变黑
			cv::Mat mcorners(corners);     //复制矩阵（避免破坏原有矩阵）
			mcorners *= (1.0 / image_sf);     //缩放角坐标
			image_points.push_back(corners);     //在image_points后插入corners，这里注意一下，此举相当于，在image图像上叠加了一个棋盘图像
			object_points.push_back(vector<cv::Point3f>());     //在object_points后插入Point3f类型函数，同理，先加上一个还没有求解到的图像，用一个空矩阵表示
																//下面这段其实我觉得我的理解有点问题，我的理解是：将输出图像所占内存大小调整到最优，获取图像直到数目达到预设值
			vector<cv::Point3f> & opts = object_points.back();     //opts即：Options,简单来说就是将输出图像的最后一位大小最优
			opts.resize(board_n);     //调整容器大小
			for (int j = 0; j < board_n; j++)
			{
				opts[j] = cv::Point3f(static_cast<float>(j / board_w), static_cast<float>(j % board_w), 0.0f);     //将三维数据存入opts中,注意，这个地方必须加强制转换，不然会出错！！！！！！
			}
			cout << "\n已收集到" << static_cast<uint>(image_points.size()) << "张棋盘图像，总共需要" << n_boards << "张棋盘图像。\n" << endl;
		}
		cv::imshow("Calibration", image);     //显示图像

											  //等待时间为30ms，如果在这个时间段内, 用户按下ESC(ASCII码为27),则跳出循环,否则,则跳出循环
		if (((cv::waitKey(30)) & 255) == 27)
			return parameter;

	}
	//结束循环
	cv::destroyWindow("Calibration");     //销毁窗口
	cout << "\n\n正在矫正相机...\n" << endl;

	//校准相机
	cv::Mat intrinsic_matrix, distortion_coeffs;     //instrinsic_matrix:线性内在参数，3*3矩阵， distortion_coeffs：畸变系数：k1、k2、p1、p2
	double err = cv::calibrateCamera(
		object_points,
		image_points,
		image_size,
		intrinsic_matrix,
		distortion_coeffs,
		cv::noArray(),
		cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT

	);     //校准相机函数，详见P582

	cout << "***Done!\n\nReprojection error is " << err;
	//计算无畸变和修正转换映射
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
	);     //计算无畸变和修正转换映射，详见P590
		   //显示矫正的后的图像
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
		);     //利用remap重新传入图像
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

//定义函数，输入变量包括：含有图像序列号的文件、棋盘的横向格数、棋盘的纵向格数、
//单目是否已校准（用于判断采用Hartlely法还是Bouguet法，这里选择Bouguet法
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
	//定义一些量
	bool displayCorners = true;
	bool showUndistores = true;
	bool isVerticalStereo = false;
	const char* point_cloud_filename = 0;
	const int maxScale = 1;
	const float squareSize = 1.f;
	FILE* f = fopen(imageList, "rt");     //定义f为打开图像列表，模式为只读
	int i, j, lr;
	int N = nx * ny;     //定义N为棋盘角点个数
	cv::Size board_sz = cv::Size(nx, ny);     //定义board_sz为size类向量
	vector<string> imageNames[2];
	vector<cv::Point3f> boardModel;
	vector<vector<cv::Point3f>> objectPoints;
	vector<vector<cv::Point2f>> points[2];
	vector<cv::Point2f> corners[2];
	bool found[2] = { false,false };
	cv::Size imageSize;
	int ddeph = -1;

	//读取含有棋盘的图片序列
	if (!f)
	{
		cout << "打不开文件" << imageList << endl;     //要是打不开或者找不到文件，那gg
		return;
	}

	//将棋盘角点坐标存入boardmodel中，其中距离（深度）信息为0
	for (i = 0; i < ny; i++)
		for (j = 0; j < nx; j++)
			boardModel.push_back(
				cv::Point3f((float)(i*squareSize), (float)(j*squareSize), 0.f));
	i = 0;
	for (;;)
	{
		char buf[1024];    //申请一个长度为1024个字节的空间，作为字符数组使用
		lr = i % 2;     //lr用以判定读取图像为左边相机还是右边相机
		cout << "\nlr=" << lr << endl;
		if (lr == 0)
			found[0] = found[1] = false;     //如果lr为则判断为左棋盘
		cout << "\nfound=" << found[0] << found[1] << endl;
		if (!fgets(buf, sizeof(buf) - 3, f))
			break;     //如果没有读到则终止循环
		size_t len = strlen(buf);     //获取buf的字符串长度，存入len中
		while (len > 0 && isspace(buf[len - 1]))     //isspace：判断是否为空格，制表符，是则返回非零值。（其实这块我不大理解）
			buf[--len] = '\0';
		if (buf[0] == '#')     //遇到#则继续
			continue;
		cv::Mat img = cv::imread(buf, 0);     //读取buf储存地址所指向的照片，存入img中
		if (img.empty())     //要是读完了，就终止循环
			break;
		imageSize = img.size();     //将img的大小存入imagesize中
		imageNames[lr].push_back(buf);     //将buf存入imagename里面
		i++;

		//如果在左棋盘没有找到棋盘，那也没必要在右棋盘去找了
		if (lr == 1 && !found[0])
			continue;
		//找棋盘
		int s;
		for (s = 1; s <= maxScale; s++)
		{
			cv::Mat timg = img;
			if (s > 1)
				resize(img, timg, cv::Size(), s, s, cv::INTER_CUBIC);     //缩放图像
			found[lr] = cv::findChessboardCorners(timg, board_sz, corners[lr]);     //本身find函数返回的就是布尔值，直接返回到found，判断找没找到
			if (found[lr] || s == maxScale)
			{
				cv::Mat mcorners(corners[lr]);  //将角点的位置输出矩阵存到mcorner中，不破坏
				mcorners *= (1. / s);     //将角点位置还原为未缩放的位置
			}
			if (found[lr])
				break;
		}
		if (displayCorners)
		{
			cout << buf << endl;
			cv::Mat cimg;
			cv::cvtColor(img, cimg, cv::COLOR_GRAY2BGR);     //将img中的图像转换为RGB图像
			cv::drawChessboardCorners(cimg, cv::Size(nx, ny), corners[lr], found[lr]);     //绘制棋盘角点
			cv::imshow("Corners", cimg);     //显示图像cimg并将其命名为Corners
			if ((cv::waitKey(0) & 255) == 27)    //如果按下esc退出
				exit(-1);
		}
		else
			cout << '.';
		if (lr == 1 && found[0] && found[1])
		{
			objectPoints.push_back(boardModel);     //将棋盘的坐标位置信息放入objectpoint中
			points[0].push_back(corners[0]);     //将左侧角点位置矩阵放入points[0]中
			points[1].push_back(corners[1]);     //将右侧角点位置矩阵放入points[1]中
		}
	}
	fclose(f);     //关闭文件

				   //立体校正
	cv::Mat  R, T, E, F;
	cout << "\n正在进行相机立体校正";
	cv::stereoCalibrate(
		objectPoints,     //objectPoints，存储标定角点在世界坐标系中的位置
		points[0],     //imagePoints1，存储标定角点在第一个摄像机下的投影后的亚像素坐标
		points[1],     //imagePoints2，存储标定角点在第二个摄像机下的投影后的亚像素坐标
		M1,     //cameraMatrix1，输入/输出型的第一个摄像机的相机矩阵
				//注意：如果CV_CALIB_USE_INTRINSIC_GUESS , CV_CALIB_FIX_ASPECT_RATIO ,CV_CALIB_FIX_INTRINSIC , or CV_CALIB_FIX_FOCAL_LENGTH其中的一个或多个标志被设置，该摄像机矩阵的一些或全部参数需要被初始化
		D1,     //distCoeffs1，第一个摄像机的输入/输出型畸变向量
		M2,     //cameraMatrix2，输入/输出型的第一个摄像机的相机矩阵
		D2,     //distCoeffs2，第一个摄像机的输入/输出型畸变向量
		imageSize,     //imageSize，图像的大小
		R,     //R，输出型，第一和第二个摄像机之间的旋转矩阵
		T,     //T，输出型，第一和第二个摄像机之间的平移矩阵
		E,     //E，输出型，本征矩阵
		F,     //F，输出型，基本矩阵
		cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH,
		/*
		CV_CALIB_FIX_INTRINSIC 如果该标志被设置，那么就会固定输入的cameraMatrix和distCoeffs不变，只求解R, T, E, F
		CV_CALIB_USE_INTRINSIC_GUESS 根据用户提供的cameraMatrix和distCoeffs为初始值开始迭代
		CV_CALIB_FIX_PRINCIPAL_POINT 迭代过程中不会改变主点的位置
		CV_CALIB_FIX_FOCAL_LENGTH 迭代过程中不会改变焦距
		CV_CALIB_FIX_ASPECT_RATIO 固定fx/fy的比值，只将fy作为可变量，进行优化计算。
		（当CV_CALIB_USE_INTRINSIC_GUESS没有被设置，fx和fy将会被忽略。只有fx/fy的比值在计算中会被用到。)
		CV_CALIB_SAME_FOCAL_LENGTH 强制保持两个摄像机的焦距相同
		CV_CALIB_ZERO_TANGENT_DIST 切向畸变保持为零
		CV_CALIB_FIX_K1, ..., CV_CALIB_FIX_K6 迭代过程中不改变相应的值。如果设置了 CV_CALIB_USE_INTRINSIC_GUESS 将会使用用户提供的初始值，否则设置为零
		CV_CALIB_RATIONAL_MODEL 畸变模型的选择，如果设置了该参数，将会使用更精确的畸变模型，distCoeffs的长度就会变成8
		*/
		cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5)
		/*TermCriteria模板类，作为迭代算法的终止条件。
		该类变量需要3个参数，一个是类型，第二个参数为迭代的最大次数，最后一个是特定的阈值。
		类型有TermCriteria::COUNT、TermCriteria::EPS、cv::TermCriteria::COUNT|cv::TermCriteria::EPS，
		分别代表着迭代终止条件为达到最大迭代次数终止，迭代到阈值终止，或者两者都作为迭代终止条件
		这里采用第三种，最大迭代次数为100，阈值为10^-5
		*/
	);
	cout << "\n搞定！按任意键可浏览图像，按ESC退出\n\n";

	//校正检查
	vector<cv::Point3f> lines[2];
	double avgErr = 0;
	int nframes = (int)objectPoints.size();
	for (i = 0; i < nframes; i++)
	{
		vector<cv::Point2f>&pt0 = points[0][i];
		vector<cv::Point2f>&pt1 = points[1][i];
		cv::undistortPoints(pt0, pt0, M1, D1, cv::Mat(), M1);    
		//根据观察到的点坐标计算理想点坐标
		/*
		src， 观察到的点坐标
		dst，在非失真和反向透视变换后输出理想点坐标。
		cameraMatrix
		distCoeffs
		R - 对象空间中的整流变换（3x3矩阵），如果矩阵为空，则使用身份转换。
		P - 新的相机矩阵（3x3）或新的投影矩阵（3x4）。
		*/
		cv::undistortPoints(pt1, pt1, M2, D2, cv::Mat(), M2);
		cv::computeCorrespondEpilines(pt0, 1, F, lines[0]);
		cv::computeCorrespondEpilines(pt1, 2, F, lines[1]);     //极线计算

		for (j = 0; j < N; j++)
		{
			double err = fabs(pt0[j].x*lines[1][j].x + pt0[j].y*lines[1][j].y + lines[1][j].z) +
				fabs(pt1[j].x*lines[0][j].x + pt1[j].y*lines[0][j].y + lines[0][j].z);
			avgErr += err;
		}
	}
	cout << "平均误差为：" << avgErr / (nframes*N) << endl;

	//计算、显示校准之后的图像
	if (showUndistores)
	{
		cv::Mat R1, R2, P1, P2, map11, map12, map21, map22, Q;
		//如果单目已校准，则采用bouguet法
		if (!useUncalibrated)
		{
			stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q, 0);     //bouguet算法
			isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));     //判断图像是垂直还是水平
																						  //矫正映射
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

		//校正并显示图像
		cv::Mat pair;
		if (!isVerticalStereo)
			pair.create(imageSize.height, imageSize.width * 2, CV_8UC3);
		else
			pair.create(imageSize.height * 2, imageSize.width, CV_8UC3);

		//进行对应
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
			ofstream o_file(rute); //输出文件流，将数据输出到文件  
			for (int i = 0; i<vdisp.rows; i++)
			{
				for (int j = 0; j<vdisp.cols; j++)
				{
					o_file << int(vdisp.at<uchar>(cv::Point(j, i))) << "   ";
				}
				o_file << "\n";
			}
			point_cloud_filename = "point_cloud.txt";//保存云点
			if (point_cloud_filename)
			{
				printf("storing the point cloud...");
				fflush(stdout);
				cv::Mat xyz;
				cv::reprojectImageTo3D(vdisp, xyz, Q, true);
				saveXYZ(point_cloud_filename, xyz);
				printf("\n");
			}

			if (!isVerticalStereo)  //水平对应或垂直对应
			{
				cv::Mat part = pair.colRange(0, imageSize.width);    //提取pair中的一列放入part中
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
			cv::imshow("对应图像", pair);
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
	cout << "\n摄像头1的内在参数为：" << intrinsic_matrix1 << "\n摄像头1的畸变参数为：" << distortion_coeffs1 << endl;
	parameter2 = Camera_calibration(9, 6, 14, 500, 0.5, 1);
	intrinsic_matrix2 = parameter2[0];
	distortion_coeffs2 = parameter2[1];
	cout << "\n摄像头2的内在参数为：" << intrinsic_matrix2 << "\n摄像头2的畸变参数为：" << distortion_coeffs2 << endl;
	const char *board_list = "../get_depth_information2/list.txt";
	StereoCalib(board_list, board_w, board_h, false, intrinsic_matrix1, intrinsic_matrix2, distortion_coeffs1, distortion_coeffs2);
	return 0;
}