#pragma once

#include <string>
#include <ros/ros.h>

namespace rovi_system
{
	// -- meta-data ---------------------------------------------------------------

	// -- experiments -------------------------------------------------------------

	std::string
	get_experiment_dir(std::string experiment_name = "");

	std::string
	make_timestamped_data_dir(const std::string& experiment_name = "", const std::string& dir = "");
	
	std::string
	make_custom_data_dir(const std::string& experiment_name, const std::string& dir);
}
