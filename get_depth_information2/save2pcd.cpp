#include <opencv2/opencv.hpp>
#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <windows.h>
#include<iostream>

using namespace std;

void savexyzwrapper(const char* filename, const cv::Mat& mat,int i)
{
	const double max_z = 1.0e4;
	FILE* fp = fopen(filename, "wt");
	fprintf(fp, "# .PCD v1.8 - Point Cloud Data file format\n");
	fprintf(fp, "VERSION 1.8\n");
	fprintf(fp, "FIELDS x y z\n");
	fprintf(fp, "SIZE 4 4 4\n");
	fprintf(fp, "TYPE F F F\n");
	fprintf(fp, "COUNT 1 1 1\n");
	fprintf(fp, "WIDTH %d \nHEIGHT 1\n", i);//随机点云要计算真是点数
	fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
	fprintf(fp, "POINTS %d\n", i);
	fprintf(fp, "DATA ascii\n");
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
	char *savePath = "F:\\Computer vision\\Practice\\get_depth_information2\\get_depth_information2\\point_cloud1.pcd";
	remove(savePath);
	rename("point_cloud1.txt", "point_cloud1.pcd");
}