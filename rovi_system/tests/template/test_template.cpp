#include <ros/ros.h>
#include <rovi_system/rovi_system.h>

int
main(int argc, char **argv)
{
	// init ROS node
	ros::init(argc, argv, "test_template");
	ros::NodeHandle nh;

	// deduce experiment directory from node name ('test_template' → 'template')
	// must be configured properly; see 'tests/template/README.md'
	auto dir_experiment = rovi_system::get_experiment_dir(); // get_experiment_dir("template")

	// let's do an experiment
	// create a time-stampted data directory (tests/template/data/XXXXXX/)
	auto dir_data = rovi_system::make_experiment_data_dir();

	// print
	std::cout << "dir_experiment: " << dir_experiment << std::endl;
	std::cout << "dir_data: " << dir_data << std::endl;

	// write some data
	;

	// exit
	return 0;
}
