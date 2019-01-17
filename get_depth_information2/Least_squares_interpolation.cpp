#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

//最小二乘拟合相关函数定义
double sum(vector<double> Vnum, int n);
double MutilSum(vector<double> Vx, vector<double> Vy, int n);
double RelatePow(vector<double> Vx, int n, int ex);
double RelateMutiXY(vector<double> Vx, vector<double> Vy, int n, int ex);
void EMatrix(vector<double> Vx, vector<double> Vy, int n, int ex, double coefficient[]);
void CalEquation(int exp, double coefficient[]);
double F(double c[], int l, int m);
double Em[11][10];

//累加
double sum(vector<double> Vnum, int n)
{
	double dsum = 0;
	for (int i = 0; i<n; i++)
	{
		dsum += Vnum[i];
	}
	return dsum;
}
//乘积和
double MutilSum(vector<double> Vx, vector<double> Vy, int n)
{
	double dMultiSum = 0;
	for (int i = 0; i<n; i++)
	{
		dMultiSum += Vx[i] * Vy[i];
	}
	return dMultiSum;
}
//ex次方和
double RelatePow(vector<double> Vx, int n, int ex)
{
	double ReSum = 0;
	for (int i = 0; i<n; i++)
	{
		ReSum += pow(Vx[i], ex);
	}
	return ReSum;
}
//x的ex次方与y的乘积的累加
double RelateMutiXY(vector<double> Vx, vector<double> Vy, int n, int ex)
{
	double dReMultiSum = 0;
	for (int i = 0; i<n; i++)
	{
		dReMultiSum += pow(Vx[i], ex)*Vy[i];
	}
	return dReMultiSum;
}
//计算方程组的增广矩阵
void EMatrix(vector<double> Vx, vector<double> Vy, int n, int ex, double coefficient[])
{
	for (int i = 1; i <= ex; i++)
	{
		for (int j = 1; j <= ex; j++)
		{
			Em[i][j] = RelatePow(Vx, n, i + j - 2);
		}
		Em[i][ex + 1] = RelateMutiXY(Vx, Vy, n, i - 1);
	}
	Em[1][1] = n;
	CalEquation(ex, coefficient);
}
//求解方程
void CalEquation(int exp, double coefficient[])
{
	for (int k = 1; k<exp; k++) //消元过程
	{
		for (int i = k + 1; i<exp + 1; i++)
		{
			double p1 = 0;

			if (Em[k][k] != 0)
				p1 = Em[i][k] / Em[k][k];

			for (int j = k; j<exp + 2; j++)
				Em[i][j] = Em[i][j] - Em[k][j] * p1;
		}
	}
	coefficient[exp] = Em[exp][exp + 1] / Em[exp][exp];
	for (int l = exp - 1; l >= 1; l--)   //回代求解
		coefficient[l] = (Em[l][exp + 1] - F(coefficient, l + 1, exp)) / Em[l][l];
}
//供CalEquation函数调用
double F(double c[], int l, int m)
{
	double sum = 0;
	for (int i = l; i <= m; i++)
		sum += Em[l - 1][i] * c[i];
	return sum;
}

double* Least_squares_interpolation(
	cv::Mat xyz_1,     //点云数据
	cv::Point2d p      //待插值点像素坐标

)
{
	int z1, z2, z3, m, n, m1, n1, i, k;
	double arry1[10], arry2[10], arry3[10], arry4[10];
	//double *points;
	double *points1 = new double[3];
	double x, y, z;
	//cv::Mat xyz_2;
	//xyz_2= xyz_1.at<cv::Vec3f>(p);
	//cout << "\n" << xyz_1.at<cv::Vec3f>(p) << "\n";
	//points = getpoints(xyz_2);
	cv::Vec3f &bgr = xyz_1.at <cv::Vec3f >(p);	
	x = bgr.val[0];
	y = bgr.val[1];
	z = bgr.val[2];
	//cout << x <<"\n"<< y <<"\n"<< z<<"\n";
	m = p.x;
	n = p.y;
	m1 = m;
	n1 = n;
	i = 0;
	//在横坐标上进行插值
	if (z == 10000)
	{
		for (k = 1; k <= 640; k++)
		{
			p.x = m++;
			p.y = n;
			cv::Vec3f &bgr = xyz_1.at <cv::Vec3f >(p);
			z1 = bgr.val[2];
			//xyz_2 = xyz_1.at<cv::Vec3f>(p);
			//points = getpoints(xyz_2);
			//z1 = points[2];
			if (z1 != 10000)
			{
				arry1[i] = z1;
				arry2[i] = m;
				//cout << "\n" << z1 << "\n";
				i++;
			}
			if (i == 5)
				break;		
		}
		for (k = 1; k <= 640; k++)
		{
			p.x = m--;
			p.y = n;
			cv::Vec3f &bgr = xyz_1.at <cv::Vec3f >(p);
			z1 = bgr.val[2];
			//xyz_2 = xyz_1.at<cv::Vec3f>(p);
			//points = getpoints(xyz_2);
			//z1 = points[2];
			if (z1 != 10000)
			{
				arry1[i] = z1;
				arry2[i] = m;
				//cout << "\n" << z1 << "\n";
				i++;
			}
			if (i == 10)
				break;
		}
	}
	double coefficient[5];
	memset(coefficient, 0, sizeof(double) * 5);
	vector<double> vx, vy;
	for (int i = 0; i<10; i++)
	{
		vx.push_back(arry1[i]);
		vy.push_back(arry2[i]);
	}
	EMatrix(vx, vy, 10, 2, coefficient);
	//printf("\n拟合方程为：y = %lf + %lfx \n", coefficient[1], coefficient[2]);
	z2 = coefficient[1] + coefficient[2] * m1;
    //在纵坐标上进行插值
	i = 0;
	m = m1;
	n = n1;
	if (z == 10000)
	{
		for (k = 1; k <= 480; k++)
		{
			p.y = n++;
			p.x = m;
			cv::Vec3f &bgr = xyz_1.at <cv::Vec3f >(p);
			z1 = bgr.val[2];
			//xyz_2 = xyz_1.at<cv::Vec3f>(p);
			//points = getpoints(xyz_2);
			//z1 = points[2];
			if (z1 != 10000)
			{
				arry3[i] = z1;
				arry4[i] = n;
				//cout << "\n" << z1 << "\n";
				i++;
			}
			if (i == 5)
				break;
		}
		for (k = 1; k <= 480; k++)
		{
			p.x = n--;
			p.x = m;
			cv::Vec3f &bgr = xyz_1.at <cv::Vec3f >(p);
			z1 = bgr.val[2];	
			//xyz_2 = xyz_1.at<cv::Vec3f>(p);
			//points = getpoints(xyz_2);
			//z1 = points[2];
			if (z1 != 10000)
			{
				arry3[i] = z1;
				arry4[i] = n;
				//cout << "\n" << z1 << "\n";
				i++;
			}
			if (i == 10)
				break;
		}
	}
	double coefficient1[5];
	memset(coefficient1, 0, sizeof(double) * 5);
	vector<double> vx1, vy1;
	for (int i = 0; i<10; i++)
	{
		vx1.push_back(arry3[i]);
		vy1.push_back(arry4[i]);
	}
	EMatrix(vx1, vy1, 10, 2, coefficient1);
	//printf("\n拟合方程为：y = %lf + %lfx \n", coefficient1[1], coefficient1[2]);
	z3 = coefficient1[1] + coefficient1[2] * n1;
	z = (z2 + z3)*0.5;
	points1[0] = x;
	points1[1] = y;
	points1[2] = z;
	return points1;
}