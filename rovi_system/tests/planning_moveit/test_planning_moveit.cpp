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

#include "test_planning_moveit.h"

int
main(int argc, char **argv)
{
	// usage:
	// roslaunch rovi_system workcell.launch objects:=false moveit:=true rviz:=true pose_robot:="-x 0.4 -y 0.55"

	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "test_planning_moveit");
	ros::NodeHandle nh;

	// parameters
	auto NUM_ITER = 50u;
	auto PICK_INDEX = (argc >= 2) ? atoi(argv[1]) : 2; // default index
	auto OBJ_NAME = std::string("bottle");

	// ------------------------------------------------------------------------------

	// setup simulation + scene
	ROS_INFO_STREAM("Setting up simulation...");
	gazebo::set_simulation(true);
	wsg50::release();
	// gazebo::projector().set(false);
	ros::Duration(5.0).sleep(); // settle

	// go home, you're drunk
	ur5::command_home();

	// spawn obstacles
	ROS_INFO_STREAM("Spawning obstacles and object...");
	rovi_system::spawn_obstacles();
	gazebo::spawn_model(OBJ_NAME, OBJ_NAME + "1", PICK_LOCATIONS[PICK_INDEX]);

	// init moveit
	ur5::moveit::init(nh);

	// load collision objects from Gazebo
	ur5::moveit::update_planning_scene_from_gazebo();
	ur5::moveit::start_scene_publisher(1); // Hz

	// ------------------------------------------------------------------------------
	
	// define pose of object in base frame with some offset; moveit will plan for b_T_tcp
	auto w_T_obj = Eigen::make_tf(PICK_LOCATIONS[PICK_INDEX]);
	auto pose_obj_tcp = geometry_msgs::make_pose(ur5::w_T_b().inverse() * w_T_obj * Eigen::make_tf({ 0, 0, 0.1 }));

	// define tolerances for planning at TCP
	// this allows end-effector to grasp object at a "free-rotating" z-axis
	auto tolerances = std::array{ std::vector(3, 0.001), std::vector{0.01, 0.01, 3.14}}; // pos [m], ori [rad]

	// ------------------------------------------------------------------------------

	// make dirs for data (test_planning_moveit/data/XXXXXX/<planner>)
	auto dir_data = rovi_system::make_timestamped_data_dirs("planning_moveit", {
		ur5::moveit::PLANNERS[Planner::RRT],
		ur5::moveit::PLANNERS[Planner::RRTConnect]
	});

	// export meta-data
	std::ofstream fs(dir_data + "/info.txt", std::ostream::out);
	fs << "object: " << OBJ_NAME << "\npick_index: " << PICK_INDEX << "\nnum_iter: " << NUM_ITER << "\nw_T_obj:\n" << w_T_obj.matrix().format({Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n"});
	fs.close();

	// ------------------------------------------------------------------------------

	// do experiments for each planner
	for (auto planner : { Planner::RRT, Planner::RRTConnect })
	{
		using Traj = trajectory_msgs::JointTrajectory;
		using Plan = rovi_planner::PlanningData<Traj>;
		using Clock = std::chrono::high_resolution_clock;
		using Time = std::chrono::time_point<Clock>;

		// data directory for planner
		auto dir_data_planner = dir_data + "/" + ur5::moveit::PLANNERS[planner];

		// do iterations per planner
		std::vector<Plan> results;
		for (auto [i, plan, t] = std::tuple{ 0, Plan(), Time()}; i < NUM_ITER and ros::ok(); ++i)
		{
			t = Clock::now();
			auto plan_moveit = ur5::moveit::plan(pose_obj_tcp, planner, "tcp", tolerances);

			plan.planning_time = std::chrono::duration<double, std::milli>(Clock::now() - t).count();
			plan.iteration = i;

			if (plan_moveit.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
			{
				// ur5::moveit::export_ctraj(*plan_moveit.trajectory_, dir_data_planner + "/traj_pre" + std::to_string(i) + ".csv"); // before trajectory simplification
				plan.traj = ur5::moveit::plan_to_jnt_traj(plan_moveit);
				plan.traj_duration = plan_moveit.trajectory_->getDuration();
				ur5::moveit::export_ctraj(*plan_moveit.trajectory_, dir_data_planner + "/traj" + std::to_string(i) + ".csv"); // after trajectory simplification
			}

			results.push_back(plan);
		}

		// export all planning data for planner
		rovi_planner::export_planning_data(results, dir_data_planner + "/plan.csv");
	}

	// ------------------------------------------------------------------------------

	ROS_WARN_STREAM("Finished experiments!");
	
	// exit cleanly (or at least try to, lol)
	ur5::moveit::terminate();

	// exit
	return 0;
}
