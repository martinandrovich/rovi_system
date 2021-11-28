#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <kdl/trajectory_composite.hpp>

namespace rovi_planner
{
	using TrajPtr = std::shared_ptr<KDL::Trajectory_Composite>;

	TrajPtr
	traj_lin(
		const std::vector<geometry_msgs::Pose>& waypoints,
		double vel_max,
		double acc_max,
		double equiv_radius
	);

	TrajPtr
	traj_par(
		const std::vector<geometry_msgs::Pose>& waypoints,
		double vel_max,
		double acc_max,
		double corner_radius,
		double equiv_radius
	);

	std::vector<Eigen::Isometry3d>
	traj_to_T(const TrajPtr& traj, double dt = 0.001);

	void
	export_traj(const TrajPtr& traj, const std::string& path, double dt = 0.001);

	void
	export_waypoints(const std::vector<geometry_msgs::Pose>& waypoints, const std::string& path);
}