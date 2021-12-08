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
	// rosrun rovi_system test_pick

	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "test_pick");
	ros::NodeHandle nh;

	// parameters
	auto NUM_ITER = 20;
	auto PLANNER = Planner::SBL;
	auto OBJ_NAME = std::string("bottle");
	auto OBJ_ID = OBJ_NAME + "1";
	auto TOLERANCES = std::array{std::vector(3, 0.001), std::vector{0.001, 0.001, 3.14}}; // pos [m], ori [rad]

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
	auto dir_data = rovi_system::make_timestamped_data_dir("pick_and_place", "pick_pipeline");
	
	// write metadata
	std::ofstream fs(dir_data + "/info.txt", std::ostream::out);
	fs << "object: " << OBJ_NAME << "\n"
	   << "num_iter: " << NUM_ITER << "\n"
	   << "planner: " << ur5::moveit::PLANNERS[PLANNER] << "\n"
	   << "tol_pos: " << TOLERANCES[0] << "\n"
	   << "tol_ori: " << TOLERANCES[1] << "\n"
	   << "pick_offset:\n" << PICK_OFFSET.translation().matrix() << "\n";
	fs.close();
	
	// experiments
	for (auto PICK_INDEX = 0; PICK_INDEX < PICK_LOCATIONS.size(); PICK_INDEX++)
	{
		// object
		auto obj_pose = PICK_LOCATIONS[PICK_INDEX];
		
		// spawn object (and delete previous if any)
		gazebo::delete_model(OBJ_ID);
		gazebo::spawn_model(OBJ_NAME, OBJ_ID, obj_pose);
		
		// update moveit
		ur5::moveit::update_planning_scene_from_gazebo();
		
		// open file
		std::ofstream fs(dir_data + "/pick_attempts_" + std::to_string(PICK_INDEX) + ".csv", std::ofstream::out);
		fs << "iteration, success" << std::endl;
		
		// perform pose estimation + planning for NUM_ITE times
		for (auto i = 0; i < NUM_ITER; ++i)
		{
			// pose estimation
			auto obj_pose_est = obj_pose;
			auto pose_obj_tcp = ur5::get_tcp_given_pose(obj_pose_est, PICK_OFFSET);
			
			// planning
			auto plan = ur5::moveit::plan(pose_obj_tcp, PLANNER, "tcp", TOLERANCES);
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
