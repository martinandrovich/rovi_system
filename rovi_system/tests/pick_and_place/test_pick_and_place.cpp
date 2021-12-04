#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <geometry_msgs/Pose.h>

#include <ur5_description/ur5_description.h>
#include <ur5_controllers/ur5_controllers.h>
#include <ur5_gazebo/ur5_gazebo.h>
#include <wsg50/wsg50.h>

#include <rovi_system/rovi_system.h>
#include <rovi_planner/planning_data.h>
#include <ur5_planner/moveit.h>
#include <ur5_planner/reachability.h>

#include <rovi_system/planning_common.h>

int
main(int argc, char **argv)
{
	// usage:
	// roslaunch rovi_system workcell.launch objects:=false moveit:=true rviz:=true pose_robot:="-x 0.4 -y 0.55"
	// rosrun rovi_system test_pick_and_place

	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "test_pick_and_place");
	ros::NodeHandle nh;

	// parameters
	auto NUM_ITER = 20;
	auto PLANNER = Planner::SBL;
	auto PICK_INDEX = 0u;
	auto OBJ_NAME = std::string("bottle");
	auto OBJ_ID = OBJ_NAME + "1";
	auto OBJ_POSE = PICK_LOCATIONS[PICK_INDEX];
	auto TOLERANCES = std::array{std::vector(3, 0.001), std::vector{0.001, 0.001, 3.14}}; // pos [m], ori [rad]
	auto MAX_PLANNING_TIME = 5.0; // [sec]
	auto MAX_PLANNING_ATTEMPTS = 100;

	// ------------------------------------------------------------------------------

	// setup simulation + scene
	ROS_INFO_STREAM("Setting up simulation...");
	gazebo::set_simulation(true);
	wsg50::release();
	ros::Duration(5.0).sleep(); // settle

	// go home, you're drunk
	ur5::command_home();

	// spawn obstacles
	ROS_INFO_STREAM("Spawning obstacles and object...");
	rovi_system::spawn_obstacles();

	// init moveit
	ur5::moveit::init(nh);
	ur5::moveit::start_scene_publisher(10); // Hz

	// config pose estimation
	// ...

	// ------------------------------------------------------------------------------

	// create data directory
	auto dir_data = rovi_system::make_timestamped_data_dir("pick_and_place", "pick_and_place");

	// write metadata
	{
	std::ofstream fs(dir_data + "/info.txt", std::ostream::out);
	fs << "object: " << OBJ_NAME << "\n"
	   << "num_iter: " << NUM_ITER << "\n"
	   << "planner: " << ur5::moveit::PLANNERS[PLANNER] << "\n"
	   << "tol_pos: " << TOLERANCES[0] << "\n"
	   << "tol_ori: " << TOLERANCES[1] << "\n"
	   << "max_planning_time: " << MAX_PLANNING_TIME << "\n"
	   << "max_planning_attempts: " << MAX_PLANNING_ATTEMPTS << "\n"
	   << "place_pos:\n" << PLACE_LOCATION.position
	   << "pick_offset:\n" << PICK_OFFSET.translation().matrix() << "\n";
	}

	// open file
	std::ofstream fs(dir_data + "/pick_and_place.csv", std::ofstream::out);
	fs << "iteration, success, phase" << std::endl;

	// experiments
	for (auto [i, phase] = std::tuple{0, std::string()}; i < NUM_ITER; i++)
	{
		// reset and settle
		gazebo::delete_model(OBJ_ID);
		ur5::command_home();
		wsg50::release(true); // block while grasping

		// respawn object and update planning scene
		gazebo::spawn_model(OBJ_NAME, OBJ_ID, OBJ_POSE);
		ur5::moveit::update_planning_scene_from_gazebo(); // update and remove attached objects

		// pose estimation
		auto obj_pose_est = OBJ_POSE;
		auto pose_obj_tcp = ur5::get_tcp_given_pose(obj_pose_est, PICK_OFFSET);

		// ------------------------------------------------------------------------------

		// planning
		phase = "pick_plan";
		auto plan_pick = ur5::moveit::plan(pose_obj_tcp, PLANNER, "tcp", TOLERANCES, MAX_PLANNING_TIME, MAX_PLANNING_ATTEMPTS);

		if (plan_pick.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{ fs  << i << ", " << 0 << ", " << phase << "\n"; continue; }

		// execute trajectory
		phase = "pick_execute";
		auto traj_pick = ur5::moveit::plan_to_jnt_traj(plan_pick);
		ur5::command_traj(traj_pick);
		ros::Duration(2.0).sleep(); // settle
		ur5::moveit::update_planning_scene_from_gazebo();

		// check that object is unmoved
		// auto [dr, dt] = geometry_msgs::pose_diff(OBJ_POSE, obj_pose_measured);
		std::cout << "OBJ_POSE: " << OBJ_POSE << std::endl;
		std::cout << "obj_pose_measured: " << gazebo::get_pose(OBJ_ID) << std::endl;

		if (false)
			{ fs  << i << ", " << 0 << ", " << phase << "\n"; continue; }

		// grasp object
		phase = "pick_gripper";
		wsg50::grasp(true); // block while grasping
		ur5::moveit::attach_object_to_ee(OBJ_ID);
		ros::Duration(1.0).sleep(); // settle

		// ------------------------------------------------------------------------------

		// ENTER_TO_CONTINUE("place");

		// desired object pose
		auto pose_obj_place_tcp = ur5::get_tcp_given_pose(PLACE_LOCATION, PICK_OFFSET);

		// planning
		phase = "place_plan";
		auto plan_place = ur5::moveit::plan(pose_obj_place_tcp, PLANNER, "tcp", TOLERANCES, MAX_PLANNING_TIME, MAX_PLANNING_ATTEMPTS);

		if (plan_place.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{ fs  << i << ", " << 0 << ", " << phase << "\n"; continue; }

		// execute trajectory
		phase = "place_execute";
		auto traj_place = ur5::moveit::plan_to_jnt_traj(plan_place);
		ur5::command_traj(traj_place);

		// release gripper
		phase = "place_gripper";
		wsg50::release(true);
		ros::Duration(1.0).sleep(); // settle

		// measure diff for {x,y} only
		std::cout << "OBJ_POSE: " << PLACE_LOCATION << std::endl;
		std::cout << "obj_pose_measured: " << gazebo::get_pose(OBJ_ID) << std::endl;

		// write to file
		fs  << i << ", " << 1 << ", " << phase << "\n";
	}

	// ------------------------------------------------------------------------------

	// exit cleanly (or at least try to, lol)
	ur5::moveit::terminate();

	// exit
	return 0;
}
