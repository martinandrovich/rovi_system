#pragma once

#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>

namespace rovi_vision::RANSACRegistrationWithICP
{
	Eigen::Isometry3d
	est_pose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_scene);

	void
	set_obj(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_obj);
}