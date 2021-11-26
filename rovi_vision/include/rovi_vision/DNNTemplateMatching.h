#pragma once

#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/Image.h>

namespace rovi_vision::DNNTemplateMatching
{
	Eigen::Isometry3d
	est_pose(const std::string& name, const sensor_msgs::Image& img);

	void
	set_template_dir_for_obj(const std::string& name, const std::string& dir);
}