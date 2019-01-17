#pragma once

#ifndef CIRCLE_H
#define CIRCLE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef struct result
{
	cv::Mat  R;
	cv::Mat  T;
	cv::Mat  E;
	cv::Mat  F;
	cv::Size imageSize;
}LINETYPE;

extern LINETYPE *camera;

result *StereoCalib
(
	int board_w,
	int board_h,
	cv::Mat M1,
	cv::Mat D1,
	cv::Mat M2,
	cv::Mat D2,
	const char *imageList,
	bool useUncalibrated,
	bool displayCorners,
	bool showUndistores,
	bool isVerticalStereo
);

void saveXYZWrapper(const char* filename, const cv::Mat& mat);

void savexyzwrapper(const char* filename, const cv::Mat& mat, int i);

int CountLines(const char* filename);

int* findcontours();

//void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
int pcd_view();

int License_verification();

//void mose_callback(int event, int x, int y, int flags, void * param);

std::vector<cv::Mat> Camera_calibration(
	int board_w,     //���̵Ŀ��
	int board_h,     //���̵ĸ߶�
	int n_boards,     //���궨ͼ�����Ŀ��������������������ȡ��Ϊ�˱�֤��������⾫�ȣ�����������Ҫ10�����ϵ�ͼ��      
	int delay,     //�����������ʱΪ1s
	double image_sf,     //���ű���Ϊ0.5
	int cap     //ѡ��������
);

void xian_de();

double* Least_squares_interpolation(cv::Mat xyz_1, cv::Point2d p);

#endif
