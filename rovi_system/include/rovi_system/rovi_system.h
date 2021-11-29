#pragma once

#include <string>
#include <initializer_list>
#include <ros/ros.h>
#include <ros_utils/geometry_msgs.h>

namespace rovi_system
{
	// -- meta-data ---------------------------------------------------------------
	
	static const struct
	{
		double LENGTH             = 0.80; // [m] (x)
		double WIDTH              = 1.20; // [m] (y)
		double HEIGHT             = 0.75; // [m] (z)
		double MASS               = 10.0; // [kg]
		geometry_msgs::Pose POSE  = geometry_msgs::make_pose({ 0.4, 0.6, 0.64 });
	} TABLE;
	
	// -- setup -------------------------------------------------------------------
	
	void
	spawn_obstacles();

	// -- experiments -------------------------------------------------------------

	std::string
	get_experiment_dir(std::string experiment_name = "");

	std::string
	make_timestamped_data_dir(const std::string& experiment_name = "", const std::string& dir = "");
	
	std::string
	make_timestamped_data_dirs(const std::string& experiment_name, const std::initializer_list<std::string>& dirs);
	
	std::string
	make_custom_data_dir(const std::string& experiment_name, const std::string& dir);
}
