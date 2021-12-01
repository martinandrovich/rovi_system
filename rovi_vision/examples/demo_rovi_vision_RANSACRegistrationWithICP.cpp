#include <iostream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>

#include <rovi_vision/RANSACRegistrationWithICP.h>

int main(int argc, char** argv)
{
	// first run:
	// ...

	// load object point cloud
	auto cloud_obj = pcl::load_cloud("/path/to/object.pcd"); // from ros_utils
	auto cloud_obj = pcl::load_cloud("package://rovi_models/models/milk/milk.pcd"); // from ros_utils

	// get point cloud from workcell
	auto cloud_sene = gazebo::kinect().get_cloud(); // from ros_utils

	// estimate pose using global registration
	auto pose = rovi_vision::RANSACRegistrationWithICP::est_pose(cloud_scene, cloud_obj);

	return 0;
}