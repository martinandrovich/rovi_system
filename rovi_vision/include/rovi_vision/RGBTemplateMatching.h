#pragma once

#include <string>
#include <Eigen/Eigen>
#include <sensor_msgs/Image.h>

namespace rovi_vision::RGBTemplateMatching
{
	Eigen::Isometry3d
	est_pose(const std::string& name, const sensor_msgs::Image& img, const bool vis = false);

	void
	set_template_dir_for_obj(const std::string& name, const std::string& dir);
}