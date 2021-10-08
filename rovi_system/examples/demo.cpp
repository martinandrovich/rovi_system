#include <iostream>
#include <string>

#include <ros/ros.h>
#include <rovi_system/rovi_system.h>

auto
TEST = [](const std::string& msg, auto&& lambda)
{
	std::cout << "# TESTING: " << msg << " ...\n";
	lambda();
	std::cout << std::endl;
};

int
main(int argc, char **argv)
{
	// init ROS node
	ros::init(argc, argv, "demo");
	ros::NodeHandle nh;

	// project_foo
	TEST("rovi_system", rovi_system::test);

	// exit
	return 0;
}
