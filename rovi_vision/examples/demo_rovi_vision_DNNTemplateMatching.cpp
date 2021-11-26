#include <iostream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>

#include <rovi_vision/DNNTemplateMatching.h>

int main(int argc, char** argv)
{
	// first run:
	// ...

	// DNN must be trained and templates must be generated for specific object
	// do so by running:
	// ...

	// get image from workcell
	auto img = gazebo::camera().get_img(); // from ros_utils

	// estimate pose using global registration
	// should the templates be at 'rovi_vision/data/DNNTemplateMatching/milk' or '/rovi_models/models/milk/template'?
	rovi_vision::DNNTemplateMatching::set_template_dir_for_obj("milk", "path/to/template/dir/for/milk"); // must be set
	auto pose = rovi_vision::DNNTemplateMatching::est_pose("milk", img);

	return 0;
}