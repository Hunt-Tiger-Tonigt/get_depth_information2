#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <iostream>//��׼C++���е�������������ͷ�ļ���  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���

bool iteration_flag = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) 
{
	if (event.getKeySym() == "space" && event.keyDown())
	{
		iteration_flag = true;
	}
}