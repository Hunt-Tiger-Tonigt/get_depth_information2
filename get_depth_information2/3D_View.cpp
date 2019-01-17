#include<F:\Computer vision\Practice\get_depth_information2\get_depth_information2\wyc.h>
#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<opencv2/opencv.hpp>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

/*
bool iteration_flag = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "m" && event.keyDown())
	{
		iteration_flag = true;
	}
}
*/

int pcd_view()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("point_cloud1.pcd", *cloud) == -1) {
		std::cerr << "open failed!" << std::endl;
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	//viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		/*
		if (iteration_flag != false)
		{
			break;
		}
		*/
	}
	return 0;
}