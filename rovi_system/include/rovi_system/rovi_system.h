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
	get_data_dir(const std::string& experiment_name = "");

	std::string
	make_experiment_data_dir(const std::string& experiment_name = "");
}
