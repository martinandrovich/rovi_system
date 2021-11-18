#include <iostream>

#include <ros/ros.h>
#include <rovi_system/rovi_system.h>

int
main(int argc, char **argv)
{
	// init ROS node
	ros::init(argc, argv, "test_reachability");
	ros::NodeHandle nh;
	
	std::cout << "Hello from test_reachability!" << std::endl;

	// exit
	return 0;
}
