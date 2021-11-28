#pragma once

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <kdl/trajectory_composite.hpp>

namespace rovi_planner
{
	template <typename TrajT>
	struct PlanningData
	{
		size_t iteration;
		double planning_time;
		double traj_duration;
		TrajT traj;
	};
	
	template <typename TrajT>
	void
	export_planning_data(const std::vector<PlanningData<TrajT>>& data, const std::string& path)
	{
		std::ofstream fs(path, std::ofstream::out);
		fs << "i, planning time [ms], traj_duration [s]" << std::endl;
		for (const auto& plan : data)
			fs << plan.iteration << ", " << plan.planning_time << ", " << plan.traj_duration << std::endl;
		fs.close();
	}
}
