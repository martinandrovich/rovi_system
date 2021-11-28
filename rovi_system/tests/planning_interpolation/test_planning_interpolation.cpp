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
#include <wsg50/wsg50.h>
#include <ur5_gazebo/ur5_gazebo.h>

#include <rovi_system/rovi_system.h>
#include <rovi_planner/rovi_planner.h>
#include <rovi_planner/planning_data.h>

#include "test_planning_interpolation.h"

int
main(int argc, char **argv)
{
	// usage:
	// test_planning_interpolation [pick_location]

	// first run:
	// roslaunch rovi_system workcell.launch objects:=false pose_robot:="-x 0.4 -y 0.55"

	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "planning_interpolation");
	ros::NodeHandle nh;

	// parameters
	auto NUM_ITER = 50u;
	auto [VEL_MAX, ACC_MAX, CORNER_RADIUS, EQUIV_RADIUS] = std::tuple{ 0.1, 0.1, 0.05, 0.001 };
	auto PICK_INDEX = (argc >= 2) ? atoi(argv[1]) : 2;

	// ------------------------------------------------------------------------------

	// setup simulation + scene
	// gazebo::projector().set(false);
	gazebo::set_simulation(true);
	wsg50::release();
	ur5::command_setpoint(ur5::Q_HOME, true);

	// ------------------------------------------------------------------------------

	// set object pose (0, 1 or 2, default = 2)
	auto pose_obj = PICK_LOCATIONS[PICK_INDEX];

	// pick offsets
	auto PRE_PICK_OFFSET = Eigen::Isometry3d({ 0.0, -0.1, 0.1 }); // {x, y, z}
	auto PICK_OFFSET = Eigen::Isometry3d({ 0.0, 0.0, 0.1 }); // {x, y, z}

	// generate list of waypoints (poses) for EE in base frame
	// VIA_POINTS etc. defined in 'test_planning_interpolation.h'
	std::vector<geometry_msgs::Pose> waypoints =
	{
		ur5::get_pose("ee"),                                      // starting pose
		ur5::get_ee_given_pose_at_tcp(VIA_POINTS["orient"]),      // orient EE and center
		ur5::get_ee_given_pose_at_tcp(VIA_POINTS["move-down"]),   // move down
		ur5::get_ee_given_pose_at_tcp(VIA_POINTS["pre-fork"]),    // pre-fork
		ur5::get_ee_given_pose_at_tcp(VIA_POINTS["fork"]),        // fork
		ur5::get_ee_given_pose_at_tcp(pose_obj, PRE_PICK_OFFSET), // pre-obj
		ur5::get_ee_given_pose_at_tcp(pose_obj, PICK_OFFSET)      // obj
	};

	// ------------------------------------------------------------------------------

	// make dir for data
	auto dir_data = rovi_system::make_timestamped_data_dir("planning_interpolation");

	// export waypoints
	rovi_planner::export_waypoints(waypoints, dir_data + "/waypoints.csv");

	// do experiments for linear and parabolic interpolation
	for (std::string method : { "lin", "par" })
	{
		using Traj = rovi_planner::TrajPtr;
		using Plan = rovi_planner::PlanningData<Traj>;
		using Clock = std::chrono::high_resolution_clock;
		using Time = std::chrono::time_point<Clock>;

		// do iterations per method
		std::vector<Plan> results;
		for (auto [i, plan, t] = std::tuple{ 0, Plan(), Time()}; i < NUM_ITER and ros::ok(); ++i)
		{
			if (method == "lin")
				t = Clock::now(), plan.traj = rovi_planner::traj_lin(waypoints, VEL_MAX, ACC_MAX, EQUIV_RADIUS);
			if (method == "par")
				t = Clock::now(), plan.traj = rovi_planner::traj_par(waypoints, VEL_MAX, ACC_MAX, CORNER_RADIUS, EQUIV_RADIUS);

			plan.planning_time = std::chrono::duration<double, std::milli>(Clock::now() - t).count();
			plan.traj_duration = plan.traj->Duration();
			plan.iteration = i;

			results.push_back(plan);
		}

		// export all planning data for method
		rovi_planner::export_planning_data(results, dir_data + "/plan_" + method + ".csv");
		rovi_planner::export_traj(results[0].traj, dir_data + "/traj_" + method + ".csv");
	}

	// ------------------------------------------------------------------------------

	// execute trajectory (demo only)

	ENTER_TO_CONTINUE("execute trajectory");

	auto traj_lin = rovi_planner::traj_lin(waypoints, VEL_MAX, ACC_MAX, EQUIV_RADIUS);
	auto traj_par = rovi_planner::traj_par(waypoints, VEL_MAX, ACC_MAX, CORNER_RADIUS, EQUIV_RADIUS);

	ur5::command_traj(traj_par, ur5::EXEC_DT);
	gazebo::spawn_model("bottle", "bottle1", pose_obj);

	// exit
	return 0;
}
