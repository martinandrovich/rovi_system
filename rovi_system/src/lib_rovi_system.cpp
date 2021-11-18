#include <rovi_system/rovi_system.h>
#include <filesystem>
#include <ros/package.h>
#include <ros_utils/std.h>

std::string
rovi_system::get_experiment_dir(std::string experiment_name)
{
	// deduce experiment name if not provided
	if (experiment_name == "")
		experiment_name = ros::this_node::getName().substr(6); // remove '/test_'

	// define directory
	const auto dir = ros::package::getPath("rovi_system") + "/tests/" + experiment_name;

	// check if directory exists; return if all good
	if (not std::filesystem::exists(dir))
		throw std::runtime_error("The experiment directory: '" + dir + "' does not exist.");
	else
		return dir;
}

std::string
rovi_system::get_data_dir(const std::string& experiment_name)
{
	return get_experiment_dir(experiment_name) + "/data";
}

std::string
rovi_system::make_experiment_data_dir(const std::string& experiment_name)
{
	const auto dir = rovi_system::get_data_dir(experiment_name) + "/" + get_timestamp();
	if (std::filesystem::create_directories(dir))
		ROS_INFO_STREAM("Created directory: " << dir);

	return dir;
}