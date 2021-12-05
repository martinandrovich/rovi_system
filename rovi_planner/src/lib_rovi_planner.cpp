#include <rovi_planner/rovi_planner.h>

#include <vector>
#include <memory>
#include <filesystem>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl/frames.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_line.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/utilities/error.h>

std::shared_ptr<KDL::Trajectory_Composite>
rovi_planner::traj_lin(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max, double acc_max, double equiv_radius)
{
	if (waypoints.size() < 2)
		throw std::invalid_argument("There must be at least two waypoints in traj_lin().");

	// convert to vector<KDL::Frame>
	std::vector<KDL::Frame> frames;
	for (const auto& pt : waypoints)
	{
		KDL::Frame frame;
		tf::poseMsgToKDL(pt, frame);
		frames.push_back(frame);
	}

	// try creating the trajectory; catch any errors
	auto traj = std::make_shared<KDL::Trajectory_Composite>();

	try
	{
		// create a trajectory segment for each waypoint defined as a path line betwen the current and next point with some velocity profile
		// http://docs.ros.org/en/melodic/api/orocos_kdl/html/classKDL_1_1Path__Line.html#a1ea3f21f577aee2a4252c5a802b6a7f2
		for (auto i = 0; i < frames.size() - 1; ++i)
		{
			// create path
			auto rot  = new KDL::RotationalInterpolation_SingleAxis();
			auto path = new KDL::Path_Line(frames[i], frames[i + 1], rot, equiv_radius);

			// define velocity profile for path
			auto vel_profile = new KDL::VelocityProfile_Trap(vel_max, acc_max);
			vel_profile->SetProfile(0, path->PathLength());

			// construct segment from path and velocity profile
			auto traj_seg = new KDL::Trajectory_Segment(path, vel_profile);
			traj->Add(traj_seg);
		}
	}
	catch (const KDL::Error& e)
	{
		std::cerr << e.GetType() << std::endl;
		std::cerr << e.Description() << std::endl;
		throw std::runtime_error("Could not plan trajectory in traj_lin().");
	}

	// return trajectory
	return traj;
}

std::shared_ptr<KDL::Trajectory_Composite>
rovi_planner::traj_par(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max, double acc_max, double corner_radius, double equiv_radius)
{
	if (waypoints.size() < 2)
		throw std::invalid_argument("There must be at least two waypoints in traj_par().");

	// convert to vector<KDL::Frame>
	std::vector<KDL::Frame> frames;
	for (const auto& pt : waypoints)
	{
		KDL::Frame frame;
		tf::poseMsgToKDL(pt, frame);
		frames.push_back(frame);
	}

	// define traj, path etc.
	// destruction is handled by traj object (when it goes out of scope)
	auto traj = std::make_shared<KDL::Trajectory_Composite>();
	auto rot  = new KDL::RotationalInterpolation_SingleAxis();
	auto path = new KDL::Path_RoundedComposite(corner_radius, equiv_radius, rot); // path composed of waypoints with rounded corners

	// try creating the trajectory; catch any errors
	try
	{
		// add all points (frames) to rounded composite path
		// between to adjecent points a Path_Line will be created, between two lines there will be rounding with the given radius with a Path_Circle
		for (const auto& pt : frames)
			path->Add(pt);

		// finish creating the path
		path->Finish();

		// define velocity profile based on path start and end (for the whole trajectory segment)
		auto vel_profile  = new KDL::VelocityProfile_Trap(vel_max, acc_max);
		vel_profile->SetProfile(0, path->PathLength());

		// add trajectory segment from path and velocity profile to final trajectory
		auto traj_seg = new KDL::Trajectory_Segment(path, vel_profile);
		traj->Add(traj_seg);
	}
	catch (const KDL::Error& e)
	{
		std::cerr << e.GetType() << std::endl;
		std::cerr << e.Description() << std::endl;
		throw std::runtime_error("Could not plan trajectory in traj_par().");
	}

	// return trajectory
	return traj;
}

std::vector<Eigen::Isometry3d>
rovi_planner::traj_to_T(const TrajPtr& traj, double dt)
{
	std::vector<Eigen::Isometry3d> vec_T;
	vec_T.reserve(int(traj->Duration()/dt));
	for (auto t = 0.; t < traj->Duration(); t += dt)
	{
		// KDL Frame
		const auto& pos = traj->Pos(t);
		// const auto& vel = traj->Vel(t);
		// const auto& acc = traj->Acc(t);

		// convert KDL Frame with position to Eigen transformaton
		static Eigen::Isometry3d T;
		tf::transformKDLToEigen(pos, T);
		vec_T.push_back(T);
	}

	return vec_T;
}

void
rovi_planner::export_waypoints(const std::vector<geometry_msgs::Pose>& waypoints, const std::string& path)
{
	// delete file if it exists
	std::filesystem::remove(path);

	for (const auto& pt : waypoints)
	{
		static Eigen::Affine3d T;
		tf::poseMsgToEigen(pt, T);
		// export each waypoint as linewise, csv, row-major matrices, append to same file
		Eigen::export_csv(T.matrix(), path, { .linewise_csv = true, .row_major = true, .output_mode = std::ofstream::app });
	}
}

void
rovi_planner::export_traj(const TrajPtr& traj, const std::string& path, double dt)
{
	// delete file if it exists
	std::filesystem::remove(path);

	for (double t = 0.0; t < traj->Duration(); t += dt)
	{
		// KDL Frame
		const auto& frame = traj->Pos(t);
		const auto& twist = traj->Vel(t);

		// convert KDL Frame to Eigen
		static auto T = Eigen::Affine3d();
		tf::transformKDLToEigen(frame, T);

		// export each waypoint as linewise, csv, row-major matrices, append to same file
		Eigen::export_csv(T.matrix(), path, { .linewise_csv = true, .row_major = true, .output_mode = std::ofstream::app });
	}
}