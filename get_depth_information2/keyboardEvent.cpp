#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <iostream>//标准C++库中的输入输出类相关头文件。  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h> //PCL中支持的点类型头文件。

bool iteration_flag = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) 
{
	if (event.getKeySym() == "space" && event.keyDown())
	{
		iteration_flag = true;
	}
}