#include <iostream>
#include <vector>
#include <tuple>

#include <ros/ros.h>
#include <rovi_planner/rovi_planner.h>
#include <ros_utils/geometry_msgs.h>

int main(int argc, char** argv)
{
	using namespace geometry_msgs;

	ros::init(argc, argv, "demo_rovi_planner");
	ros::NodeHandle nh;

	// define waypoints
	std::vector<geometry_msgs::Pose> waypoints =
	{
		make_pose({ 0.0, 0.0, 0.0 }, { 0,      0, 0 }),
		make_pose({ 0.0, 0.0, 1.0 }, { 0, M_PI/2, 0 }),
		make_pose({ 2.0, 1.0, 0.0 }, { 0,      0, 0 }),
	};

	// parameters
	auto [VEL_MAX, ACC_MAX, CORNER_RADIUS, EQUIV_RADIUS] = std::tuple{ 0.1, 0.1, 0.05, 0.001 };

	// trajectories
	auto traj_lin = rovi_planner::traj_lin(waypoints, VEL_MAX, ACC_MAX, EQUIV_RADIUS);
	auto traj_par = rovi_planner::traj_par(waypoints, VEL_MAX, ACC_MAX, CORNER_RADIUS, EQUIV_RADIUS);

	// print info
	std::cout << "traj_lin->Duration(): " << traj_lin->Duration() << "\n";
	std::cout << "traj_par->Duration(): " << traj_par->Duration() << "\n";
	
	// discretize linear trajectory with some dt into a vector of transforms (Eigen::Isometry3d)
	auto dt    = 1.0;
	auto dur   = traj_lin->Duration();
	auto vec_T = rovi_planner::traj_to_T(traj_lin, dt);

	for (auto [t, T] = std::tuple{ 0., vec_T.begin()}; t < dur; T++, t += dt )
		std::cout << t << "/" << dur << " [sec]:\n\n" << T->matrix() << "\n\n";

	// export trajectories to be plotted in MATLAB
	// ...

	return 0;
}
