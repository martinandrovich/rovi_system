#include <iostream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <ros_utils/pcl.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <rovi_vision/RANSACRegistrationWithICP.h>

// #include <rovi_vision/RANSACRegistrationWithICP.h>

int main(int argc, char** argv)
{
	// first run:
	// roslaunch rovi_system workcell.launch objects:=false projector:=false pose_robot:="-x 0.4 -y 0.55"

	// init ROS node
	ros::init(argc, argv, "demo_RANSACRegistrationWithICP");
	ros::NodeHandle nh;

	// spawn milk
	gazebo::spawn_model("milk", "milk1", {0.4, 1.0, 0.755});

	// load object point cloud
	auto cloud_obj = pcl::load_cloud<pcl::PointXYZ>("package://rovi_models/models/milk/milk.pcd");

	// auto cloud_kinect = gazebo::kinect().get_cloud();
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZ>());
	// pcl::fromROSMsg(*cloud_kinect, *cloud_scene);

	// get point cloud from workcell as PCL
	auto cloud_scene = gazebo::kinect().get_cloud<pcl::PointCloud<pcl::PointXYZ>>();

	// estimate pose using global registration
	// rovi_vision::RANSACRegistrationWithICP::set_obj(cloud_obj);
	auto pose = rovi_vision::RANSACRegistrationWithICP::est_pose(cloud_scene, cloud_obj);
	std::cout << "pose: " << pose.matrix() << std::endl;

	return 0;
}