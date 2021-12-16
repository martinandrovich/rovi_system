#pragma once

#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>

namespace rovi_vision::RANSACRegistrationWithICP
{
	Eigen::Isometry3d
	est_pose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_obj = nullptr);

	void
	set_obj(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_obj);

	// variables
	inline bool visualize = true;
	inline float leaf = 0.01f;
	inline int max_ransac = 1'000'000;
	inline int num_of_threads = 8;
}