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
(   int board_w,     //棋盘的宽度
	int board_h,     //棋盘的高度
	cv::Mat M1, 
	cv::Mat D1, 
	cv::Mat M2, 
	cv::Mat D2,
	const char *imageList,     //图像文件列表
	bool useUncalibrated,     //是否使用未校准方法
	bool displayCorners,     //是否显示角点
	bool showUndistores,     //是否显示校正后的图像
	bool isVerticalStereo     //判断图像是垂直还是水平
)

{
	const char* point_cloud_filename = 0;
	const char* point_cloud_filename1 = 0;
	const int maxScale = 1;
	const float squareSize = 1.f;
	FILE* f = fopen(imageList, "rt");     //定义f为打开图像列表，模式为只读
	int N = board_w * board_h;     //定义N为棋盘角点个数
	cv::Size board_sz = cv::Size(board_w, board_h);     //定义board_sz为size类向量
	int jsq = 0;
	int i, j, lr;
	vector<string> imageNames[2];
	vector<cv::Point3f> boardModel;
	vector<vector<cv::Point3f>> objectPoints;
	vector<vector<cv::Point2f>> points[2];
	vector<cv::Point2f> corners[2];
	bool found[2] = { false,false };
	cv::Size imageSize;

	//读取含有棋盘的图片序列
	if (!f)
	{
		cout << "打不开文件" << imageList << endl;     //要是打不开或者找不到文件，那gg
		return NULL;
	}

	//将棋盘角点坐标存入boardmodel中，其中距离（深度）信息为0
	for (i = 0; i < board_h; i++)
		for (j = 0; j < board_w; j++)
			boardModel.push_back(
				cv::Point3f((float)(i*squareSize), (float)(j*squareSize), 0.f));
	i = 0;
	for (;;)
	{
		char buf[1024];    //申请一个长度为1024个字节的空间，作为字符数组使用
		lr = i % 2;     //lr用以判定读取图像为左边相机还是右边相机
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
			cv::drawChessboardCorners(cimg, cv::Size(board_w, board_h), corners[lr], found[lr]);     //绘制棋盘角点
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
	system("color 67");

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
			/*
			cameraMatrix1– 第一个相机矩阵.
			distCoeffs1– 第一个相机畸变参数.
			cameraMatrix2– 第二个相机矩阵.
			distCoeffs2– 第二个相机畸变参数.
			imageSize– 用于校正的图像大小.
			R– 第一和第二相机坐标系之间的旋转矩阵。
			T– 第一和第二相机坐标系之间的平移矩阵.
			R1– 输出第一个相机的3x3矫正变换(旋转矩阵) .
			R2– 输出第二个相机的3x3矫正变换(旋转矩阵) .
			P1–在第一台相机的新的坐标系统(矫正过的)输出 3x4 的投影矩阵
			P2–在第二台相机的新的坐标系统(矫正过的)输出 3x4 的投影矩阵
			Q–输出深度视差映射矩
			*/
			isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));     //判断图像是垂直还是水平
			//矫正映射
			initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);
			/*
			cameraMatrix——输入的摄像机内参数矩阵
			distCoeffs——输入的摄像机畸变系数矩阵
			R——输入的第一和第二相机坐标系之间的旋转矩阵
			newCameraMatrix——输入的校正后的3X3摄像机矩阵
			size——摄像机采集的无失真图像尺寸
			m1type——map1的数据类型，可以是CV_32FC1或CV_16SC2
			map1——输出的X坐标重映射参数
			map2——输出的Y坐标重映射参数
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

		//校正并显示图像
		cv::Mat pair;
		if (!isVerticalStereo)
			pair.create(imageSize.height, imageSize.width * 2, CV_8UC3);    //图像水平
		else
			pair.create(imageSize.height * 2, imageSize.width, CV_8UC3);    //图像垂直

																			//进行对应
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
			ofstream o_file(rute); //输出文件流，将数据输出到文件  
			for (int i = 0; i < vdisp.rows; i++)
			{
				for (int j = 0; j < vdisp.cols; j++)
				{
					o_file << int(vdisp.at<uchar>(cv::Point(j, i))) << "   ";
				}
				o_file << "\n";
			}
			point_cloud_filename = "point_cloud.txt";//保存云点
			point_cloud_filename1 = "point_cloud1.txt";//保存云点
			if (true)
			{
				printf("storing the point cloud...");
				fflush(stdout);
				cv::Mat xyz;
				cv::reprojectImageTo3D(vdisp, xyz, Q, true);
				cv::Point2d p;
				p.x = 320; p.y = 240;
				cout << "中心点的深度为：" << xyz.at<cv::Vec3f>(p) << "\n";
				saveXYZWrapper(point_cloud_filename, xyz);
				int m;
				m = CountLines(point_cloud_filename);
				savexyzwrapper(point_cloud_filename1, xyz,m);
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
			if ((cv::waitKey() & 255) == 97)
			{
				cv::destroyWindow("3D Viewer"); 
				int c=pcd_view();
			}
			if ((cv::waitKey() & 255) == 27)
				break;
		}
	}
	cv::destroyWindow("对应图像");
	cv::destroyWindow("Corners");
	result *Stereo = new result;
	Stereo->R = R;
	Stereo->T = T;
	Stereo->E = E;
	Stereo->F = F;
	Stereo->imageSize = imageSize;
	return Stereo;
}
