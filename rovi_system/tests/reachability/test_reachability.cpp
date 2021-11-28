#include <iostream>
#include <array>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <rovi_system/rovi_system.h>
#include <ur5_planner/moveit.h>
#include <ur5_planner/reachability.h>

int
main(int argc, char **argv)
{
	using namespace rovi_system;
	using namespace geometry_msgs;

	// first run:
	// roslaunch ur5_gazebo ur5.launch ee:=wsg50 gazebo:=false moveit:=true rviz:=true
	// or use 'test_reachability.launch'

	// init ROS node
	ros::init(argc, argv, "test_reachability");
	ros::NodeHandle nh;

	// parameters
	const auto OBJ_NAME   = "bottle";
	const auto OBJ_POSE   = make_pose({ 0.5, 1.0, TABLE.HEIGHT });
	const auto VISUALIZE  = (argc > 1) ? std::string(argv[1]) == "true" : false;
	const auto RESOLUTION = VISUALIZE ? 16 : 180;

	// ------------------------------------------------------------------------------

	// sleep a bit to let moveit startup
	ros::Duration(2.0).sleep();

	// init moveit
	ur5::moveit::init(nh);
	ur5::moveit::start_scene_publisher(100);

	// setup planning scene
	// add collision objects to planning scene
	ur5::moveit::add_cobjs(
	{
		{ "table",  TABLE.POSE },
		{ "bottle", make_pose({ 0.50, 1.00, 0.75 }) }
	}, "rovi_models");

	// let everything settle
	ros::Duration(2.0).sleep();

	// ------------------------------------------------------------------------------

	// generate base poses (discretize table)
	std::vector<Pose> base_poses;
	for (double y = 0; y < TABLE.WIDTH; y += 0.1)
		for (double x = 0.; x < TABLE.LENGTH; x += 0.1)
			base_poses.push_back(make_pose({ x, y, TABLE.HEIGHT - 0.01 }));

	ROS_INFO_STREAM("Generated " << base_poses.size() << " base poses for reachability test...");
	ros::Duration(1.0).sleep();

	// perform reachability test for each of the grasping orientations (top and side)
	for (std::string grasp_orientation : { "top", "side" })
	{
		// make experiment directory (once)
		// static auto dir_data = rovi_system::make_custom_data_dir("reachability", "/" + grasp_orientation);
		static auto dir_data = rovi_system::make_timestamped_data_dir("reachability");

		// define grasping transformation
		auto T_grasp = (grasp_orientation == "top") ? GRASP_TOP_AT_TCP(0.2) : GRASP_SIDE_AT_TCP(0.1);

		// // iterate different base poses
		std::vector<ur5::moveit::ReachabilityData> results;
		for (const auto& pose : base_poses)
		{
			auto result = ur5::moveit::reachability(pose, OBJ_POSE, T_grasp, RESOLUTION, VISUALIZE);
			results.push_back(result);
		}

		// write results to file
		auto path = dir_data + "/" + grasp_orientation + ".csv";
		std::ofstream fs(path, std::ofstream::out);

		fs << "x [m], y [m], plausible states" << std::endl;
		for (const auto& result : results)
			fs << result.pos_base[0] << ", " << result.pos_base[1] << ", " << result.plausible_states << "\n";
		fs.close();
		ROS_INFO_STREAM("Data was written to '" << path);
	}

	// cleanup
	ur5::moveit::terminate();

	// exit
	return 0;
}
