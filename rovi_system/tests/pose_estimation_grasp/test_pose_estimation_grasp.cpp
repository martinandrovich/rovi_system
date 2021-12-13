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
	// for each pick location, estimate the pose using rovi_vision and try to plan to pick
	// using a MoveIt planner - log binary (binomial) data

	// usage:
	// roslaunch rovi_system workcell.launch objects:=false moveit:=true rviz:=true pose_robot:="-x 0.4 -y 0.55"
	// rosrun rovi_system test_pose_estimation_grasp

	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "test_pose_estimation_grasp");
	ros::NodeHandle nh;

	// parameters
	auto METHOD = "RANSACRegistrationWithICP";
	auto NUM_ITER = 20;
	auto PLANNER = Planner::PRM;
	auto OBJ_NAME = std::string("milk");
	auto OBJ_ID = OBJ_NAME + "1";
	auto TOLERANCES = std::array{std::vector(3, 0.001), std::vector{0.001, 0.001, 3.14}}; // pos [m], ori [rad]
	auto MAX_PLANNING_TIME = 1.0; // [sec]
	auto MAX_PLANNING_ATTEMPTS = 10;

	// ------------------------------------------------------------------------------

	// setup simulation + scene
	ROS_INFO_STREAM("Setting up simulation...");
	gazebo::set_simulation(true);
	wsg50::release();
	ros::Duration(5.0).sleep(); // settle

	// go home, you're drunk
	ur5::command_home();

	// spawn object at default pose
	gazebo::spawn_model(OBJ_NAME, OBJ_ID, PICK_LOCATIONS[1]);

	// init moveit
	ur5::moveit::init(nh);
	ur5::moveit::start_scene_publisher(10); // Hz

	// config pose estimation
	// ...

	// load poses
	std::vector<geometry_msgs::Pose> poses = { PICK_LOCATIONS[0], PICK_LOCATIONS[1] };

	// ------------------------------------------------------------------------------

	// create data directory (pose_estimation_grasp/data/METHOD/...)
	auto dir_data = rovi_system::make_timestamped_data_dir("pose_estimation_grasp", METHOD);

	// write metadata
	std::ofstream fs(dir_data + "/info.txt", std::ostream::out);
	fs << "object: " << OBJ_NAME << "\n"
	   << "method: " << METHOD << "\n"
	   << "planner: " << ur5::moveit::PLANNERS[PLANNER] << "\n"
	   << "num_iter: " << NUM_ITER << "\n"
	   << "num_poses: " << poses.size() << "\n"
	   << "tol_pos: " << TOLERANCES[0] << "\n"
	   << "tol_ori: " << TOLERANCES[1] << "\n"
	   << "pick_offset:\n" << PICK_OFFSET.translation().matrix() << "\n";
	fs.close();

	// experiments
	for (auto pose_idx = 0; pose_idx < poses.size(); pose_idx++)
	{
		// move object to new pose
		const auto& pose_obj = poses[pose_idx];
		gazebo::move_model(OBJ_ID, pose_obj);

		// update moveit
		ur5::moveit::update_planning_scene_from_gazebo();

		// open file
		std::ofstream fs(dir_data + "/pick_attempts_" + std::to_string(pose_idx) + ".csv", std::ofstream::out);
		fs << "iteration, success" << std::endl;

		// perform pose estimation + planning for NUM_ITE times
		for (auto i = 0; i < NUM_ITER; ++i)
		{
			// pose estimation
			auto pose_obj_est = pose_obj;

			// planning
			auto pose_obj_tcp = ur5::get_tcp_given_pose(pose_obj_est, PICK_OFFSET);
			auto plan = ur5::moveit::plan(pose_obj_tcp, PLANNER, "tcp", TOLERANCES, MAX_PLANNING_TIME, MAX_PLANNING_ATTEMPTS);
			
			// assume that object is graspable if planner is able to find a plan
			auto success = (plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

			// write to file
			fs  << i << ", " << success << "\n";
		}
	}

	// ------------------------------------------------------------------------------

	// exit cleanly (or at least try to, lol)
	ur5::moveit::terminate();

	// exit
	return 0;
}
