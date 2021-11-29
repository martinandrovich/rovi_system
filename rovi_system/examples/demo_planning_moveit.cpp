#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>

#include <ur5_gazebo/ur5_gazebo.h>
#include <ur5_controllers/ur5_controllers.h>
#include <wsg50/wsg50.h>

#include <rovi_system/rovi_system.h>
#include <rovi_planner/planning_data.h>
#include <ur5_planner/moveit.h>
#include <ur5_planner/reachability.h>

int
main(int argc, char **argv)
{
	// usage:
	// roslaunch rovi_system workcell.launch objects:=false moveit:=true rviz:=true pose_robot:="-x 0.4 -y 0.55"

	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "demo_planning_moveit");
	ros::NodeHandle nh;

	// ------------------------------------------------------------------------------

	// setup simulation + scene
	ROS_INFO_STREAM("Setting up simulation...");
	gazebo::set_simulation(true);
	wsg50::release();
	// gazebo::projector().set(false);
	ros::Duration(1.0).sleep(); // settle

	// go home, you're drunk
	ur5::command_home();

	// define pose of object in base frame with some offset (moveit will plan for b_T_tcp)
	auto pose_obj = geometry_msgs::make_pose({ 0.15, 1.05, 0.75 });
	auto w_T_obj = Eigen::make_tf(pose_obj);
	auto pose_obj_tcp = geometry_msgs::make_pose(ur5::w_T_b().inverse() * w_T_obj * Eigen::make_tf({ 0, 0, 0.1 }));

	// spawn obstacles and object
	rovi_system::spawn_obstacles();
	gazebo::spawn_model("bottle", "bottle1", pose_obj);

	// init moveit and load collision objects from Gazebo
	ur5::moveit::init(nh);
	ur5::moveit::update_planning_scene_from_gazebo();
	ur5::moveit::start_scene_publisher(1); // Hz

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("plan");

	// define tolerances for planning at TCP
	// this allows end-effector to grasp object at a "free-rotating" z-axis
	auto tolerances = std::array{
		std::vector(3, 0.001),        // pos {xyz} [m]
		std::vector{0.01, 0.01, 3.14} // ori {rpy} [rad]
	};

	// configure planner (if needed)
	// ur5::moveit::set_planner_config(Planner::EST, "property", 1.0);

	// make plan and trajectory
	auto plan = ur5::moveit::plan(pose_obj_tcp, Planner::EST, "tcp", tolerances, 5.0, 100);
	auto traj = ur5::moveit::plan_to_jnt_traj(plan);

	ENTER_TO_CONTINUE("execute trajectory");

	ur5::command_traj(traj);

	// ------------------------------------------------------------------------------

	// exit cleanly (or at least try to, lol)
	ur5::moveit::terminate();

	// exit
	return 0;
}
