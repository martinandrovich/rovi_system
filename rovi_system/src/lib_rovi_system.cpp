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
		throw std::invalid_argument("The experiment directory: '" + dir + "' does not exist in get_experiment_dir().");
	
	// create sub-directoires
	std::filesystem::create_directories(dir + "/data");
	std::filesystem::create_directories(dir + "/img");

	return dir;
}

std::string
rovi_system::make_timestamped_data_dir(const std::string& experiment_name, const std::string& dir)
{

	if (not dir.empty() and dir[0] != '/')
		throw std::invalid_argument("The provided directory '" + dir + "' must begin with '/' in make_timestamped_data_dir().");

	auto dir_data = get_experiment_dir(experiment_name) + "/data" + dir + "/" + get_timestamp(); // time-stamped data directory
	auto dir_img  = get_experiment_dir(experiment_name) + "/img"; // img dir (for all experiments)

	if (std::filesystem::create_directories(dir_data))
		ROS_INFO_STREAM("Created time-stamped data directory: " << dir_data);

	if (std::filesystem::create_directories(dir_img))
		ROS_INFO_STREAM("Created img directory: " << dir_img);

	return dir_data;
}

std::string
rovi_system::make_custom_data_dir(const std::string& experiment_name, const std::string& dir)
{
	if (dir.empty() or dir[0] != '/')
		throw std::invalid_argument("The provided directory '" + dir + "' cannot be empty and must begin with '/' in make_custom_data_dir().");
	
	auto dir_data = get_experiment_dir(experiment_name) + "/data" + dir;
	if (std::filesystem::create_directories(dir_data))
		ROS_INFO_STREAM("Created data directory: " << dir_data);
		
	return dir_data;
}