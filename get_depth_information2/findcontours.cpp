#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include<opencv2/opencv.hpp>
#include<iostream>

#pragma warning(disable:4244)

using namespace std;

//�����ṹ��AreaCmp���������
struct AreaCmp {
	AreaCmp(const vector<float>& _areas) : areas(&_areas) {}
	bool operator()(int a, int b) const { return (*areas)[a] > (*areas)[b]; }      //������ʽ����Ϊ����
	const vector<float>* areas;
};

int* findcontours()
{
	cv::Mat img_1, img_edge, img_color;
	img_1 = cv::imread("666.jpg", cv::IMREAD_GRAYSCALE);     //��ȡ��Ƭ����Ϊ8λ��ͨ��ͼ��
	cv::threshold(img_1, img_edge, 125, 255, cv::THRESH_BINARY);     //�ο���https://blog.csdn.net/kksc1099054857/article/details/75041215
	//cv::imshow("wyc", img_edge);     //����ʱר��
	vector< vector< cv::Point > > contours;     //�洢�����㣬�ο���https://blog.csdn.net/Ahuuua/article/details/80593388
	vector< cv::Vec4i > hierarchy;     //�洢�������ṹ
	cv::findContours(
		img_edge,     //Ѱ��������ͼ�񣬱���Ϊ8λ��ͨ��ͼ��
		contours,     //�������
		hierarchy,     //���������
		cv::RETR_LIST,
		/*
		������ȡ��������ѡ�����£�
		RETR_EXTERNAL:��ʾֻ����������������������������hierarchy[i][2]=hierarchy[i][3]=-1
		RETR_LIST:��ȡ������������������list�У����������������ȼ���ϵ
		RETR_CCOMP : ��ȡ��������������������֯��˫��ṹ(two - level hierarchy), ����Ϊ��ͨ�����Χ�߽磬�β�λ�ڲ�߽�
		RETR_TREE : ��ȡ�������������½�����״�����ṹ
		RETR_FLOODFILL����ˮ��䷨
		*/
		cv::CHAIN_APPROX_SIMPLE
		/*
		CV_CHAIN_APPROX_NONE ��������߽������������������㵽contours������
		CV_CHAIN_APPROX_SIMPLE �����������Ĺյ���Ϣ�������������յ㴦�ĵ㱣����contours�����ڣ��յ���յ�֮��ֱ�߶��ϵ���Ϣ�㲻�豣��
		CV_CHAIN_APPROX_TC89_L1��CV_CHAIN_APPROX_TC89_KCOSʹ��teh-Chinl chain �����㷨
		*/
	);
	cout << "��⵽��������Ϊ: " << contours.size() << endl;

	//��ȡ���������
	vector<int> sortIdx(contours.size());
	vector<float> areas(contours.size());
	cv::Mat boundary;
	int aims[4][2];
	int up = 0, down = 0, left = 0, right = 0;
	for (int n = 0; n < (int)contours.size(); n++) {
		sortIdx[n] = n;
		areas[n] = contourArea(contours[n], false);
	}

	std::sort(sortIdx.begin(), sortIdx.end(), AreaCmp(areas));     //�������Ĵ�С����

	for (int n = 0; n < (int)sortIdx.size(); n++)
	{
		int idx = sortIdx[n];
		cv::cvtColor(img_1, img_color, cv::COLOR_GRAY2BGR);     //���Ҷ�ͼת��ΪRGBͼ
		cv::drawContours(
			img_color,      //Ҫ����������ͼ��
			contours,     //���������������ÿ�������������һ��point����
			idx,     //ָ��Ҫ���������ı�ţ�����Ǹ�������������е�����
			cv::Scalar(0, 0, 255),   //�����������õ���ɫ   
			2,      //�����������ߵĴ�ϸ������Ǹ������������ڲ������
			cv::LINE_AA,      //�����������ߵ���ͨ��,4/8/AA
			hierarchy,      //���ڲ㼶�Ŀ�ѡ������ֻ�л��Ʋ�������ʱ�Ż��õ�
			0//������������߼����������ֻ��hierarchy��Ч��ʱ�����Ч
			 //maxLevel=0��������������������ͬһ�ȼ������������������������������ڵ�����
			 //maxLevel=1, ��������������ͬһ�ȼ����������������ӽڵ㡣
			 //maxLevel=2����������������ͬһ�ȼ����������������ӽڵ��Լ��ӽڵ���ӽڵ�
		);
		cout << "�� " << idx << " ���������������Ϊ��" << areas[idx] <<
			", �����������ص�Ϊ��" << contours[idx].size() << "\n" << "���У�" << endl;
		cv::imshow("F:\\Computer vision\\Practice\\Findcontours\\x64\\Release\\Findcontours.exe", img_color);
		//Ѱ���������ĸ�����		
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
		//д����˵�����λ��
		aims[0][0] = boundary.at<int>(miny1, 0);
		aims[0][1] = boundary.at<int>(miny1, 1);
		//д����Ͷ˵�����λ��
		aims[1][0] = boundary.at<int>(maxy1, 0);
		aims[1][1] = boundary.at<int>(maxy1, 1);
		//д������˵�����λ��
		aims[2][0] = boundary.at<int>(minx1, 0);
		aims[2][1] = boundary.at<int>(minx1, 1);
		//д�����Ҷ˵�����λ��
		aims[3][0] = boundary.at<int>(maxx1, 0);
		aims[3][1] = boundary.at<int>(maxx1, 1);
		cout << "�������λ��Ϊ��" << aims[0][0] << "," << aims[0][1] << "\n";
		cout << "��Ͷ�����λ��Ϊ��" << aims[1][0] << "," << aims[1][1] << "\n";
		cout << "���������λ��Ϊ��" << aims[2][0] << "," << aims[2][1] << "\n";
		cout << "���Ҷ�����λ��Ϊ��" << aims[3][0] << "," << aims[3][1] << "\n\n";
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
	cout << "����������ʾ���\n";
	
	return 0;
}
