#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include<opencv2/opencv.hpp>
#include<iostream>

#pragma warning(disable:4244)

using namespace std;

//创建结构体AreaCmp（面积排序）
struct AreaCmp {
	AreaCmp(const vector<float>& _areas) : areas(&_areas) {}
	bool operator()(int a, int b) const { return (*areas)[a] > (*areas)[b]; }      //将排序方式设置为降序
	const vector<float>* areas;
};

int* findcontours()
{
	cv::Mat img_1, img_edge, img_color;
	img_1 = cv::imread("666.jpg", cv::IMREAD_GRAYSCALE);     //读取相片并存为8位单通道图像
	cv::threshold(img_1, img_edge, 125, 255, cv::THRESH_BINARY);     //参看：https://blog.csdn.net/kksc1099054857/article/details/75041215
	//cv::imshow("wyc", img_edge);     //测试时专用
	vector< vector< cv::Point > > contours;     //存储轮廓点，参看：https://blog.csdn.net/Ahuuua/article/details/80593388
	vector< cv::Vec4i > hierarchy;     //存储轮廓树结构
	cv::findContours(
		img_edge,     //寻找轮廓的图像，必须为8位单通道图像
		contours,     //输出轮廓
		hierarchy,     //输出轮廓树
		cv::RETR_LIST,
		/*
		轮廓提取方法，可选择如下：
		RETR_EXTERNAL:表示只检测最外层轮廓，对所有轮廓设置hierarchy[i][2]=hierarchy[i][3]=-1
		RETR_LIST:提取所有轮廓，并放置在list中，检测的轮廓不建立等级关系
		RETR_CCOMP : 提取所有轮廓，并将轮廓组织成双层结构(two - level hierarchy), 顶层为连通域的外围边界，次层位内层边界
		RETR_TREE : 提取所有轮廓并重新建立网状轮廓结构
		RETR_FLOODFILL：洪水填充法
		*/
		cv::CHAIN_APPROX_SIMPLE
		/*
		CV_CHAIN_APPROX_NONE 保存物体边界上所有连续的轮廓点到contours向量内
		CV_CHAIN_APPROX_SIMPLE 仅保存轮廓的拐点信息，把所有轮廓拐点处的点保存入contours向量内，拐点与拐点之间直线段上的信息点不予保留
		CV_CHAIN_APPROX_TC89_L1，CV_CHAIN_APPROX_TC89_KCOS使用teh-Chinl chain 近似算法
		*/
	);
	cout << "检测到轮廓总数为: " << contours.size() << endl;

	//获取轮廓的面积
	vector<int> sortIdx(contours.size());
	vector<float> areas(contours.size());
	cv::Mat boundary;
	int aims[4][2];
	int up = 0, down = 0, left = 0, right = 0;
	for (int n = 0; n < (int)contours.size(); n++) {
		sortIdx[n] = n;
		areas[n] = contourArea(contours[n], false);
	}

	std::sort(sortIdx.begin(), sortIdx.end(), AreaCmp(areas));     //将轮廓的大小排序

	for (int n = 0; n < (int)sortIdx.size(); n++)
	{
		int idx = sortIdx[n];
		cv::cvtColor(img_1, img_color, cv::COLOR_GRAY2BGR);     //将灰度图转换为RGB图
		cv::drawContours(
			img_color,      //要绘制轮廓的图像
			contours,     //所有输入的轮廓，每个轮廓被保存成一个point向量
			idx,     //指定要绘制轮廓的编号，如果是负数，则绘制所有的轮廓
			cv::Scalar(0, 0, 255),   //绘制轮廓所用的颜色   
			2,      //绘制轮廓的线的粗细，如果是负数，则轮廓内部被填充
			cv::LINE_AA,      //绘制轮廓的线的连通性,4/8/AA
			hierarchy,      //关于层级的可选参数，只有绘制部分轮廓时才会用到
			0//绘制轮廓的最高级别，这个参数只有hierarchy有效的时候才有效
			 //maxLevel=0，绘制与输入轮廓属于同一等级的所有轮廓即输入轮廓和与其相邻的轮廓
			 //maxLevel=1, 绘制与输入轮廓同一等级的所有轮廓与其子节点。
			 //maxLevel=2，绘制与输入轮廓同一等级的所有轮廓与其子节点以及子节点的子节点
		);
		cout << "第 " << idx << " 条轮廓，像素面积为：" << areas[idx] <<
			", 连接所用像素点为：" << contours[idx].size() << "\n" << "其中：" << endl;
		cv::imshow("F:\\Computer vision\\Practice\\Findcontours\\x64\\Release\\Findcontours.exe", img_color);
		//寻找轮廓的四个顶点		
		int x, y, minx = 998, maxx = 0, miny = 998, maxy = 0, minx1, maxx1, miny1, maxy1;
		boundary = cv::Mat(contours[idx]);
		for (int i = 0; i < contours[idx].size(); i++)
		{

			x = boundary.at<int>(i, 0);
			if (minx > x)
			{
				minx = x;
				minx1 = i;
			}
			if (maxx < x)
			{
				maxx = x;
				maxx1 = i;
			}
			y = boundary.at<int>(i, 1);
			if (miny > y)
			{
				miny = y;
				miny1 = i;
			}
			if (maxy < y)
			{
				maxy = y;
				maxy1 = i;
			}
		}
		//写入最顶端的像素位置
		aims[0][0] = boundary.at<int>(miny1, 0);
		aims[0][1] = boundary.at<int>(miny1, 1);
		//写入最低端的像素位置
		aims[1][0] = boundary.at<int>(maxy1, 0);
		aims[1][1] = boundary.at<int>(maxy1, 1);
		//写入最左端的像素位置
		aims[2][0] = boundary.at<int>(minx1, 0);
		aims[2][1] = boundary.at<int>(minx1, 1);
		//写入最右端的像素位置
		aims[3][0] = boundary.at<int>(maxx1, 0);
		aims[3][1] = boundary.at<int>(maxx1, 1);
		cout << "最顶端像素位置为：" << aims[0][0] << "," << aims[0][1] << "\n";
		cout << "最低端像素位置为：" << aims[1][0] << "," << aims[1][1] << "\n";
		cout << "最左端像素位置为：" << aims[2][0] << "," << aims[2][1] << "\n";
		cout << "最右端像素位置为：" << aims[3][0] << "," << aims[3][1] << "\n\n";
		char key;
		key = (char)cv::waitKey();
		if (key == 's')
		{
			int *aim = new int[8];
			aim[0] = aims[0][0];
			aim[1] = aims[0][1];
			aim[2] = aims[1][0];
			aim[3] = aims[1][1];
			aim[4] = aims[2][0];
			aim[5] = aims[2][1];
			aim[6] = aims[3][0];
			aim[7] = aims[3][1];
			return aim;
			break;
		}

	}
	cout << "所有轮廓显示完毕\n";
	
	return 0;
}
