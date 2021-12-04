#include <fstream>

#include <ros/ros.h>
#include <rovi_system/rovi_system.h>

int
main(int argc, char **argv)
{
	// init ROS node
	ros::init(argc, argv, "test_template");
	ros::NodeHandle nh;

	// the experiment name is 'template'
	// deduce experiment directory from node name ('test_template' → 'template')
	auto dir_experiment = rovi_system::get_experiment_dir(); // get_experiment_dir("template")

	// let's do an experiment
	// create a time-stampted data directory (tests/template/data/XXXXXX/)
	auto dir_data = rovi_system::make_timestamped_data_dir("template");

	// optionally add a sub-directory (tests/template/data/MYSUBDIR/XXXXXX/)
	rovi_system::make_timestamped_data_dir("template", "MYSUBDIR");

	// or create a custom directory for data (tests/template/data/my/custom/path/)
	rovi_system::make_custom_data_dir("template", "my/custom/path/");
	
	// ------------------------------------------------------------------------------

	// data generation
	auto f = [](auto x){ return 2 * std::pow(x, 2) + x/2.; };

	// write data to file
	std::ofstream fs(dir_data + "/data.csv", std::ofstream::out);
	fs << "x [s], y [m]" << std::endl;
	for (auto x = 0.0; x <= 10.0; x += 0.1)
		fs << x << ", " << f(x) << std::endl;
	fs.close();
	
	ROS_INFO_STREAM("Data was written to '" << dir_data << "/data.csv'.");

	// exit
	return 0;
}
