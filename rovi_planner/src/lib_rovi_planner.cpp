#include <rovi_planner/rovi_planner.h>

#include <vector>
#include <memory>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
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
		frames.push_back(std::move(frame));
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
			const auto rot  = new KDL::RotationalInterpolation_SingleAxis();
			const auto path = new KDL::Path_Line(frames[i], frames[i + 1], rot, equiv_radius);

			// define velocity profile for path
			const auto vel_profile = new KDL::VelocityProfile_Trap(vel_max, acc_max);
			vel_profile->SetProfile(0, path->PathLength());

			// construct segment from path and velocity profile
			const auto traj_seg = new KDL::Trajectory_Segment(path, vel_profile);
			traj->Add(traj_seg);
		}
	}
	catch (const KDL::Error& e)
	{
		throw std::runtime_error("Could not plan trajectory in traj_lin().");
	}
	
	// return trajectory
	return traj;
}

std::vector<Eigen::Isometry3d>
rovi_planner::traj_to_T(const std::shared_ptr<KDL::Trajectory_Composite>& traj, double dt)
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