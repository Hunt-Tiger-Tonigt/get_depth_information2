#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <opencv2/opencv.hpp>

using namespace std;

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
				cv::cvtColor(image1, img_grayr, cv:: COLOR_BGR2GRAY);
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
	cv::destroyWindow("Undistored");
	parameter.push_back(intrinsic_matrix);
	parameter.push_back(distortion_coeffs);
	capture.release();
	capture1.release();
	return parameter;
}