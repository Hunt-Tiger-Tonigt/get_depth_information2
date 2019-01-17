#include <opencv2/opencv.hpp>
#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <windows.h>
#include<iostream>

using namespace std;

//保存点云数据（x，y为坐标，z为深度）
void saveXYZWrapper(const char* filename, const cv::Mat& mat)
{
	const double max_z = 1.0e4;
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
				continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
	}
	fclose(fp);
}
