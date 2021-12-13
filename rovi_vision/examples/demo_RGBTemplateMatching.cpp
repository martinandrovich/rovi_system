#include <iostream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>

#include <rovi_vision/RGBTemplateMatching.h>

int main(int argc, char** argv)
{
	// first run:
	// ...

	// init ROS node
	ros::init(argc, argv, "demo_RGBTemplateMatching");
	ros::NodeHandle nh;

	// DNN must be trained and templates must be generated for specific object
	// do so by running:
	// ...

	// get image from workcell
	auto img = gazebo::camera().get_img(); // from ros_utils

	// estimate pose using global registration
	// should the templates be at 'rovi_vision/data/RGBTemplateMatching/milk' or '/rovi_models/models/milk/template'?
	auto dir_templates = ros::package::find("package://rovi_vision/data/templates/milk/");
	std::cout << "dir_templates: " << dir_templates << std::endl;

	rovi_vision::RGBTemplateMatching::set_template_dir_for_obj("milk", dir_templates); // must be set
	auto pose = rovi_vision::RGBTemplateMatching::est_pose("milk", img, true);

	std::cout << "pose:\n\n" << pose.matrix() << std::endl;

	return 0;
}