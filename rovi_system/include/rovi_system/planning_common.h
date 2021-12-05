#pragma once

#include <string>
#include <array>
#include <unordered_map>

#include <ros_utils/ros_utils.h>
#include <rovi_system/rovi_system.h>

// this files defines all the common metadata for the planning tests
// such as the via-points, pick locations etc.

using namespace geometry_msgs;
using namespace rovi_system;

// waypoints for TCP (in world frame)
static auto VIA_POINTS = std::unordered_map<std::string, geometry_msgs::Pose>
{
	{ "orient",    make_pose({ 0.65, TABLE.WIDTH/2,       1.20 }) },
	{ "move-down", make_pose({ 0.65, TABLE.WIDTH/2,       1.00 }) },
	{ "pre-fork",  make_pose({ 0.65, TABLE.WIDTH/2 + 0.1, 0.80 }) },
	{ "fork",      make_pose({ 0.65, TABLE.WIDTH/2 + 0.2, 0.80 }) },
};

// locations for object in pick area (in world frame)
static auto PICK_LOCATIONS = std::array
{
	make_pose({ 0.15, 1.05, 0.75 }),
	make_pose({ 0.40, 1.05, 0.75 }),
	make_pose({ 0.65, 1.05, 0.75 }),
};

static auto PLACE_LOCATION = // in world frame
	make_pose({ 0.70, 0.10, 0.75 });

static auto PICK_OFFSET = Eigen::Isometry3d({ 0, 0, 0.1 }); // grasp transformation (offset)
static auto PRE_PICK_OFFSET = Eigen::Isometry3d({ 0.0, -0.1, 0.1 }); // {x, y, z}