#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <geometry_msgs/Pose.h>

#include <rovi_system/rovi_system.h>
#include <rovi_planner/rovi_planner.h>

int main(int argc, char** argv)
{
	// this code does a simple KDL-based trajectory generation and exports the data

	// usage:
	// rosrun rovi_system test_planning_interpolation_example_waypoints

	ros::init(argc, argv, "test_planning_interpolation_example_waypoints");
	ros::NodeHandle nh;

	// define waypoints
	std::vector<geometry_msgs::Pose> waypoints =
	{
		geometry_msgs::make_pose({ 0.0, 0.0, 0.0 }),
		geometry_msgs::make_pose({ 1.0, 0.0, 0.0 }),
		geometry_msgs::make_pose({ 2.0, 1.0, 0.0 }),
		geometry_msgs::make_pose({ 1.0, 2.0, 0.0 }),
	};

	// params
	double VEL_MAX = 0.2, ACC_MAX = 0.05, CORNER_RADIUS = 0.2, EQUIV_RADIUS = 0.01;

	// generate trajectories
	auto traj_lin = rovi_planner::traj_lin(waypoints, VEL_MAX, ACC_MAX, EQUIV_RADIUS);
	auto traj_par = rovi_planner::traj_par(waypoints, VEL_MAX, ACC_MAX, CORNER_RADIUS, EQUIV_RADIUS);

	// export trajectories/waypoints to be plotted in MATLAB
	auto dir_data = rovi_system::make_custom_data_dir("planning_interpolation", "example_waypoints");
	
	rovi_planner::export_traj(traj_lin, dir_data + "/traj_lin.csv", 0.01);
	rovi_planner::export_traj(traj_par, dir_data + "/traj_par.csv", 0.01);
	rovi_planner::export_waypoints(waypoints, dir_data + "/waypoints.csv");
	
	// export durations
	std::ofstream fs(dir_data + "/dur.txt", std::ofstream::out);
	fs << traj_lin->Duration() << "\n" << traj_par->Duration();

	return 0;
}
