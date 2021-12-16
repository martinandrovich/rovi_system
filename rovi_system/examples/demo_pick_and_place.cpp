#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <ros_utils/pcl.h>

#include <ur5_description/ur5_description.h>
#include <ur5_controllers/ur5_controllers.h>
#include <ur5_gazebo/ur5_gazebo.h>
#include <wsg50/wsg50.h>

#include <rovi_system/rovi_system.h>
#include <rovi_vision/RANSACRegistrationWithICP.h>
#include <ur5_planner/moveit.h>

int
main(int argc, char **argv)
{
	// usage:
	// roslaunch rovi_system workcell.launch objects:=false moveit:=true pose_robot:="-x 0.4 -y 0.55"
	// rosrun rovi_system demo_pick_and_place [obj_pos_x] [obj_pos_y]

	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "demo_pick_and_place");
	ros::NodeHandle nh;

	// object position from args
	auto pos_x = (argc >= 2) ? atof(argv[1]) : 0.15;
	auto pos_y = (argc >= 3) ? atof(argv[2]) : 1.05;

	// parameters
	auto PLANNER = Planner::PRM;
	auto OBJ_NAME = std::string("milk");
	auto OBJ_ID = OBJ_NAME + "1";
	auto OBJ_POSE = geometry_msgs::make_pose({ pos_x, pos_y, 0.75 });
	auto PLACE_POSE = geometry_msgs::make_pose({ 0.70, 0.10, 0.76 });
	auto PICK_OFFSET = Eigen::Isometry3d({ 0, 0, 0.1 });
	auto TOLERANCES_PICK = std::array{std::vector(3, 0.001), std::vector{0.001, 0.001, 3.14}}; // pos [m], ori [rad]
	auto TOLERANCES_PLACE = std::array{std::vector(3, 0.001), std::vector{0.001, 0.001, 3.14}}; // pos [m], ori [rad]
	auto MAX_PLANNING_TIME = 1.0; // [sec]
	auto MAX_PLANNING_ATTEMPTS = 10;

	// ------------------------------------------------------------------------------

	// setup simulation + scene
	ROS_INFO_STREAM("Setting up simulation...");
	gazebo::set_simulation(true);

	// go home, you're drunk
	ur5::command_home();
	wsg50::release();
	auto EE_HOME = ur5::get_pose("ee");

	// spawn obstacles and object
	ROS_INFO_STREAM("Spawning obstacles and object...");
	rovi_system::spawn_obstacles();
	gazebo::spawn_model(OBJ_NAME, OBJ_ID, OBJ_POSE);

	// init moveit
	ur5::moveit::init(nh);
	ur5::moveit::update_planning_scene_from_gazebo();
	ur5::moveit::start_scene_publisher(100); // Hz

	// config pose estimation
	rovi_vision::RANSACRegistrationWithICP::visualize = false;
	rovi_vision::RANSACRegistrationWithICP::num_of_threads = 6;
	// rovi_vision::RANSACRegistrationWithICP::max_ransac = 1'000'000; // num iterations
	rovi_vision::RANSACRegistrationWithICP::max_ransac = 500'000; // num iterations

	auto cloud_obj = pcl::load_cloud<pcl::PointXYZ>("package://rovi_models/models/milk/milk.pcd");
	rovi_vision::RANSACRegistrationWithICP::set_obj(cloud_obj);

	// ------------------------------------------------------------------------------

	{ // PICK

		// get point cloud from workcell as PCL and estimate pose
		auto cloud_scene = gazebo::kinect().get_cloud<pcl::PointCloud<pcl::PointXYZ>>();
		auto w_T_obj = rovi_vision::RANSACRegistrationWithICP::est_pose(cloud_scene, cloud_obj);

		// auto obj_pose_est = OBJ_POSE; // ground truth
		auto obj_pose_est = geometry_msgs::make_pose(w_T_obj);
		auto pose_obj_tcp = ur5::get_tcp_given_pose(obj_pose_est, PICK_OFFSET);

		// planning
		auto plan = ur5::moveit::plan(pose_obj_tcp, PLANNER, "tcp", TOLERANCES_PICK, MAX_PLANNING_TIME, MAX_PLANNING_ATTEMPTS);

		if (plan.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{ ROS_ERROR_STREAM("Failed to plan to pick the object."); exit(-1); }

		// execute trajectory and update gazebo
		auto traj = ur5::moveit::plan_to_jnt_traj(plan);
		ur5::command_traj(traj);
		ros::Duration(2.0).sleep();

		// grasp
		wsg50::grasp(true); // block while grasping

		// attach object to EE
		ur5::moveit::update_planning_scene_from_gazebo();
		ur5::moveit::attach_object_to_ee(OBJ_ID, { "tip_l", "tip_r" }); // specify touch_links
	}

	// ------------------------------------------------------------------------------

	{ // PLACE

		// desired object pose
		auto pose_obj_tcp = ur5::get_tcp_given_pose(PLACE_POSE, PICK_OFFSET);

		// planning
		auto plan = ur5::moveit::plan(pose_obj_tcp, PLANNER, "tcp", TOLERANCES_PLACE, MAX_PLANNING_TIME, MAX_PLANNING_ATTEMPTS);

		if (plan.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{ ROS_ERROR_STREAM("Failed to plan to place the object."); exit(-1); }

		// execute trajectory
		auto traj = ur5::moveit::plan_to_jnt_traj(plan);
		ur5::command_traj(traj);

		// release gripper
		wsg50::release(true);
		ros::Duration(1.0).sleep(); // settle
		ur5::moveit::update_planning_scene_from_gazebo(); // update planning scene and remove attached objects
	}

		// ------------------------------------------------------------------------------

	{ // PLAN HOME

		// planning
		auto plan = ur5::moveit::plan(EE_HOME, PLANNER, "ee");

		if (plan.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{ ROS_ERROR_STREAM("Failed to plan to go home."); exit(-1); }

		// execute trajectory
		auto traj = ur5::moveit::plan_to_jnt_traj(plan);
		ur5::command_traj(traj);

		ros::Duration(1.0).sleep(); // settle
		ur5::moveit::update_planning_scene_from_gazebo(false);
	}

	// ------------------------------------------------------------------------------

	// exit cleanly (or at least try to, lol)
	ur5::moveit::terminate();

	// exit
	return 0;
}
