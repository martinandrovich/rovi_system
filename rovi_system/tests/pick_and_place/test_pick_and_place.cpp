#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <ros_utils/pcl.h>
#include <geometry_msgs/Pose.h>

#include <ur5_description/ur5_description.h>
#include <ur5_controllers/ur5_controllers.h>
#include <ur5_gazebo/ur5_gazebo.h>
#include <wsg50/wsg50.h>

#include <rovi_system/rovi_system.h>
#include <rovi_system/planning_common.h>
#include <rovi_planner/planning_data.h>
#include <rovi_vision/RANSACRegistrationWithICP.h>
#include <ur5_planner/moveit.h>
#include <ur5_planner/reachability.h>

int
main(int argc, char **argv)
{
	// usage:
	// roslaunch rovi_system workcell.launch objects:=false moveit:=true rviz:=true pose_robot:="-x 0.4 -y 0.55"
	// rosrun rovi_system test_pick_and_place [pick_index]

	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "test_pick_and_place");
	ros::NodeHandle nh;

	// parameters
	auto NUM_ITER = 10;
	auto PLANNER = Planner::PRM;
	auto PICK_INDEX = (argc >= 2) ? atoi(argv[1]) : 2; // default index (0, 1, or 2)
	auto OBJ_NAME = std::string("milk");
	auto OBJ_ID = OBJ_NAME + "1";
	auto OBJ_POSE = PICK_LOCATIONS[PICK_INDEX];
	auto TOLERANCES_PICK = std::array{std::vector(3, 0.001), std::vector{0.001, 0.001, 0.001}}; // pos [m], ori [rad]
	auto TOLERANCES_PLACE = std::array{std::vector(3, 0.001), std::vector{0.001, 0.001, 3.14}}; // pos [m], ori [rad]
	auto MAX_PLANNING_TIME = 1.0; // [sec]
	auto MAX_PLANNING_ATTEMPTS = 10;
	auto ALLOWED_POSE_ERROR_RADIUS = 0.05; // [m]
	auto MOVE_MODEL_BEFORE_PICK = true; // useful if PC is shit

	// ------------------------------------------------------------------------------

	// setup simulation + scene
	ROS_INFO_STREAM("Setting up simulation...");
	gazebo::set_simulation(true);

	// go home, you're drunk
	ur5::command_home();
	wsg50::release();

	// spawn obstacles and object
	ROS_INFO_STREAM("Spawning obstacles and object...");
	rovi_system::spawn_obstacles();
	gazebo::spawn_model(OBJ_NAME, OBJ_ID, OBJ_POSE);

	// init moveit
	ur5::moveit::init(nh);
	ur5::moveit::start_scene_publisher(100); // Hz

	// config pose estimation
	rovi_vision::RANSACRegistrationWithICP::visualize = false;
	rovi_vision::RANSACRegistrationWithICP::num_of_threads = 8;
	rovi_vision::RANSACRegistrationWithICP::max_ransac = 1'000'000; // num iterations

	auto cloud_obj = pcl::load_cloud<pcl::PointXYZ>("package://rovi_models/models/milk/milk.pcd");
	rovi_vision::RANSACRegistrationWithICP::set_obj(cloud_obj);

	// ------------------------------------------------------------------------------

	// create data directory
	auto dir_data = rovi_system::make_timestamped_data_dir("pick_and_place");

	// write metadata
	{
	std::ofstream fs(dir_data + "/info.txt", std::ostream::out);
	fs << "object: " << OBJ_NAME << "\n"
	   << "num_iter: " << NUM_ITER << "\n"
	   << "planner: " << ur5::moveit::PLANNERS[PLANNER] << "\n"
	   << "tol_pos: " << TOLERANCES_PICK[0] << "\n"
	   << "tol_ori: " << TOLERANCES_PICK[1] << "\n"
	   << "max_planning_time: " << MAX_PLANNING_TIME << "\n"
	   << "max_planning_attempts: " << MAX_PLANNING_ATTEMPTS << "\n"
	   << "pick_index:\n" << PICK_INDEX << "\n"
	   << "pick_pos:\n" << OBJ_POSE.position << "\n"
	   << "place_pos:\n" << PLACE_LOCATION.position << "\n"
	   << "pick_offset:\n" << PICK_OFFSET.translation().matrix() << "\n";
	}

	// open file
	std::ofstream fs(dir_data + "/pick_and_place.csv", std::ofstream::out);
	fs << "iteration, success, phase" << std::endl;

	// experiments
	for (auto [i, phase] = std::tuple{0, std::string()}; i < NUM_ITER; i++)
	{
		// ENTER_TO_CONTINUE("next");

		{ // RESET
			wsg50::release(true); // block while grasping
			ur5::command_home();
			gazebo::move_model(OBJ_ID, OBJ_POSE);
			ur5::moveit::update_planning_scene_from_gazebo(); // update and remove attached objects
		}

		// ------------------------------------------------------------------------------

		// ENTER_TO_CONTINUE("pick");

		{ // PICK

			// get point cloud from workcell as PCL and estimate pose
			auto cloud_scene = gazebo::kinect().get_cloud<pcl::PointCloud<pcl::PointXYZ>>();
			auto w_T_obj = rovi_vision::RANSACRegistrationWithICP::est_pose(cloud_scene, cloud_obj);
			auto obj_pose_est = geometry_msgs::make_pose(w_T_obj);
			// auto obj_pose_est = OBJ_POSE;
			auto pose_obj_tcp = ur5::get_tcp_given_pose(obj_pose_est, PICK_OFFSET);

			std::cout << "pos: " << geometry_msgs::read_pose(obj_pose_est).pos << std::endl;
			std::cout << "rpy: " << geometry_msgs::read_pose(obj_pose_est).rpy << std::endl;

			// planning
			phase = "pick";
			auto plan = ur5::moveit::plan(pose_obj_tcp, PLANNER, "tcp", TOLERANCES_PICK, MAX_PLANNING_TIME, MAX_PLANNING_ATTEMPTS);

			if (plan.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
				{ fs  << i << ", " << 0 << ", " << phase << "\n"; continue; }

			if (MOVE_MODEL_BEFORE_PICK)
				gazebo::move_model(OBJ_ID, PLACE_LOCATION);

			// execute trajectory and update gazebo
			auto traj = ur5::moveit::plan_to_jnt_traj(plan);
			ur5::command_traj(traj);

			if (MOVE_MODEL_BEFORE_PICK)
				gazebo::move_model(OBJ_ID, OBJ_POSE);

			// grasp
			wsg50::grasp(true); // block while grasping

			// attach object to EE
			ur5::moveit::update_planning_scene_from_gazebo();
			ur5::moveit::attach_object_to_ee(OBJ_ID, { "tip_l", "tip_r" }); // specify touch_links
			// ros::Duration(1.0).sleep(); // settle
		}
		// ------------------------------------------------------------------------------

		// ENTER_TO_CONTINUE("place");

		{ // PLACE

			// desired object pose
			auto pose_obj_tcp = ur5::get_tcp_given_pose(PLACE_LOCATION, PICK_OFFSET);

			// planning
			// use different tolerances for place, since orientation doesn't matter
			phase = "place";
			auto plan = ur5::moveit::plan(pose_obj_tcp, PLANNER, "tcp", TOLERANCES_PLACE, MAX_PLANNING_TIME, MAX_PLANNING_ATTEMPTS);

			if (plan.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
				{ fs  << i << ", " << 0 << ", " << phase << "\n"; continue; }

			// execute trajectory
			phase = "place_execute";
			auto traj = ur5::moveit::plan_to_jnt_traj(plan);
			ur5::command_traj(traj);

			// release gripper
			wsg50::release(true);
			ros::Duration(1.0).sleep(); // settle
			ur5::moveit::update_planning_scene_from_gazebo(false);

			// read pose of object
			auto pose_measured = gazebo::get_pose(OBJ_ID);
			auto diff_xy = geometry_msgs::diff_xy(pose_measured, PLACE_LOCATION);
			auto success = std::abs(diff_xy) < ALLOWED_POSE_ERROR_RADIUS;

			// write to file
			fs  << i << ", " << success << ", " << phase << ", " << diff_xy << "\n";
		}
	}

	// ------------------------------------------------------------------------------

	// exit cleanly (or at least try to, lol)
	ur5::moveit::terminate();

	// exit
	return 0;
}
