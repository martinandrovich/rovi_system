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
	auto cloud_scene = gazebo::get_point_cloud("/optional/gazebo/topic"); // from ros_utils

	// estimate pose using global registration
	auto pose = rovi_vision::RANSACRegistrationWithICP::est_pose(cloud_scene, cloud_obj);

	// estimate pose using global registration with pre-set object
	rovi_vision::RANSACRegistrationWithICP::set_obj(cloud_obj);
	auto pose = rovi_vision::RANSACRegistrationWithICP::est_pose(cloud_scene);

	return 0;
}