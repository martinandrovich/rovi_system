#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <geometry_msgs/Pose.h>
#include <ur5_gazebo/ur5_gazebo.h>
#include <rovi_planner/rovi_planner.h>
#include <rovi_system/rovi_system.h>
#include <rovi_system/planning_common.h>

int
main(int argc, char **argv)
{
	// this code is used to generate graphics in Gazebo for the presentation of
	// the KDL-based interpolation tests

	// usage:
	// roslaunch rovi_system workcell.launch objects:=false pose_robot:="-x 0.4 -y 0.55"
	// rosrun rovi_system test_planning_interpolation_viapoints

	// init ROS node
	ros::init(argc, argv, "test_planning_interpolation_viapoints");
	ros::NodeHandle nh;

	// visualize via-points using red spheres

	gazebo::spawn_model("bottle", "bottle1", PICK_LOCATIONS[0]);
	gazebo::spawn_model("bottle", "bottle2", PICK_LOCATIONS[1]);
	gazebo::spawn_model("bottle", "bottle3", PICK_LOCATIONS[2]);

	gazebo::spawn_model("sphere", "sphere0", ur5::get_pose("tcp", "world")); // tcp pose in world frame
	gazebo::spawn_model("sphere", "sphere1", VIA_POINTS["orient"]);
	gazebo::spawn_model("sphere", "sphere2", VIA_POINTS["move-down"]);
	gazebo::spawn_model("sphere", "sphere3", VIA_POINTS["pre-fork"]);
	gazebo::spawn_model("sphere", "sphere4", VIA_POINTS["fork"]);

	ENTER_TO_CONTINUE("finish screenshot in Gazebo");
	return 0;
}