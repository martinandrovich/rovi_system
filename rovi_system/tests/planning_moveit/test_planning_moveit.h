#pragma once

#include <string>
#include <array>
#include <unordered_map>

#include <ros_utils/geometry_msgs.h>
#include <rovi_system/rovi_system.h>

// this files defines all the meta-data for test_planning_moveit
// such as the via-points, pick locations etc.

using namespace geometry_msgs;
using namespace rovi_system;

// locations for object in pick area (in world frame)
static auto PICK_LOCATIONS = std::array
{
	make_pose({ 0.15, 1.05, 0.75 }),
	make_pose({ 0.40, 1.05, 0.75 }),
	make_pose({ 0.65, 1.05, 0.75 }),
};
