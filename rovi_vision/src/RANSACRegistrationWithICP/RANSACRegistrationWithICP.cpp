#include "rovi_vision/RANSACRegistrationWithICP.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/spin_image.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/common/transforms.h>
#include <pcl/common/random.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <thread>
#include <limits>
#include <future>

namespace rovi_vision::RANSACRegistrationWithICP
{

const float leaf = 0.01f;
const int max_ransac = 1000000;
const int num_of_threads = 8;
const Eigen::Matrix4d lhs_coord = (Eigen::Matrix4d() <<  0., 0., 1., 0., -1., 0., 0., 0., 0.,-1., 0., 0., 0., 0., 0., 1.).finished();
const Eigen::Matrix4d rhs_coord = (Eigen::Matrix4d() <<  1., 0., 0., 0., 0., 0., 1., 0., 0., 1., 0., 0., 0., 0., 0., 1.).finished();
const Eigen::Translation3d trans(0.4, 1.96, 1.28);
const Eigen::Quaterniond quat(0.663876251364229, 0.24394620385982696, 0.23465572697772968, -0.6668547535209687);
const Eigen::Vector4f min_pt (0.f, 0.85f, 0.775f, 1.0f);
const Eigen::Vector4f max_pt (0.8f, 1.4f, 1.5f, 1.0f);
const bool vis = true;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZ>());

std::tuple<int, Eigen::Matrix4f> RANSAC
(
	const std::vector<std::tuple<double, int, int>> & correspondences,
	pcl::common::UniformGenerator<int> * generator,
	const int start,
	const int end
);

Eigen::Isometry3d
est_pose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_scene_, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_obj_)
{
	if (cloud_obj_) // called with an object, don't use cached
		rovi_vision::RANSACRegistrationWithICP::set_obj(cloud_obj_);
		
	ROS_INFO_STREAM("Estimating pose using RANSACRegistrationWithICP...");

	// cache scene
	*cloud_scene = *cloud_scene_;

	// transform the cloud to gazebos coordinate system
	{
		Eigen::Affine3d w_T_c = trans * quat;
		w_T_c = w_T_c * lhs_coord;
		pcl::transformPointCloud(*cloud_scene, *cloud_scene, w_T_c);
	}

	// remove plane
	{
		// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// pcl::SACSegmentation<pcl::PointXYZ> seg;
		// seg.setOptimizeCoefficients (true);
		// seg.setModelType(pcl::SACMODEL_PLANE);
		// seg.setMethodType(pcl::SAC_RANSAC);
		// seg.setDistanceThreshold (0.025);
		// seg.setInputCloud(cloud_scene);
		// seg.segment(*inliers, *coefficients);

		// pcl::ExtractIndices<pcl::PointXYZ> extract;
		// extract.setInputCloud(cloud_scene);
		// extract.setIndices(inliers);
		// extract.setNegative(true);
		// extract.filter(*cloud_scene);
	}

	// cropbox
	{
		pcl::CropBox<pcl::PointXYZ> cropbox(true);
		cropbox.setInputCloud (cloud_scene);
		cropbox.setMin(min_pt);
		cropbox.setMax(max_pt);
		cropbox.filter(*cloud_scene);
	}

	// statistical outlier removal
	{
		// pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		// sor.setInputCloud(cloud_scene);
		// sor.setMeanK(20);
		// sor.setStddevMulThresh(2.0);
		// sor.filter(*cloud_scene);
	}

	// voxel
	{
		pcl::VoxelGrid<pcl::PointXYZ> voxel;
		voxel.setInputCloud(cloud_scene);
		voxel.setLeafSize(leaf, leaf, leaf);
		voxel.filter(*cloud_scene);
	}

	// calculate the normal vectors to the cloud
	pcl::PointCloud<pcl::Normal>::Ptr cloud_scene_normal(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_object_normal(new pcl::PointCloud<pcl::Normal>());
	{
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;

		// compute normal scene
		ne.setInputCloud(cloud_scene);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_scene(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree_scene);
		ne.setNumberOfThreads(num_of_threads);
		ne.setViewPoint(0.4f, 1.96f, 1.28f);
		ne.setRadiusSearch(leaf*2.5f);
		ne.compute(*cloud_scene_normal);

		// compute normals object
		ne.setInputCloud(cloud_object);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_object(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree_object);
		ne.setNumberOfThreads(num_of_threads);
		ne.setViewPoint(0.4f, 1.96f, 1.28f);
		ne.setRadiusSearch(leaf*2.5f);
		ne.compute(*cloud_object_normal);
	}


	// now we may store the features of what is left of the scene
	pcl::PointCloud<pcl::Histogram<153>>::Ptr feature_scene(new pcl::PointCloud<pcl::Histogram<153>>());
	pcl::PointCloud<pcl::Histogram<153>>::Ptr feature_object(new pcl::PointCloud<pcl::Histogram<153>>());
	{
		pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153>> sie;

		// cloud scene
		sie.setInputCloud(cloud_scene);
		sie.setInputNormals(cloud_scene_normal);
		sie.setRadiusSearch(leaf*2.5f);
		sie.compute(*feature_scene);

		// cloud object
		sie.setInputCloud(cloud_object);
		sie.setInputNormals(cloud_object_normal);
		sie.setRadiusSearch(leaf*2.5f);
		sie.compute(*feature_object);
	}

	// compute the correspondences
	std::vector<std::tuple<double, int, int>> correspondences;
	for(auto i = 0; i < feature_scene->points.size(); i++)
	{
		correspondences.push_back(std::tuple<double, int, int>(std::numeric_limits<double>::max(), 0, 0));
		for(auto j = 0; j < feature_object->points.size(); j++)
		{
			double sum = 0;
			for(auto k = 0; k < feature_scene->points[0].descriptorSize(); k++)
				sum += std::pow(feature_scene->points[i].histogram[k] - feature_object->points[j].histogram[k], 2);
			if(sum < std::get<0>(correspondences[i]))
				correspondences[i] = std::tuple<double, int, int>(sum, i, j);
		}
	}

	// 1 million iterations
	int x = 0;
	pcl::common::UniformGenerator<int> generator(0, correspondences.size()-1);
	generator.setSeed(time(NULL));
	std::vector<std::future<std::tuple<int, Eigen::Matrix4f>>> handles;

	for(auto i = 0; i < num_of_threads; i++)
	{
		auto handle = std::async(std::launch::async, &RANSAC, correspondences, &generator, x, x + int(max_ransac/num_of_threads));
		handles.emplace_back(std::move(handle));
		x += (max_ransac/num_of_threads);
	}

	// prepare for iteration
	int max_inliers = std::numeric_limits<int>::min();
	Eigen::Matrix4f best_transform;

	// iterate through the handles
	for(auto & handle : handles)
	{
		// wait until handle is finished with its task
		while (not handle.valid());

		// get the handle_value
		// auto handle_val = handle.get();
		// if(max_inliers < std::get<0>(handle_val))
		
		// check if number of inliers is larger than the current
		if (auto [inliers, tf] = handle.get(); max_inliers < inliers)
		{
			max_inliers = inliers;
			best_transform = tf;
		}
	}

	// transform the scene with the best estimate
	transformPointCloud(*cloud_scene, *cloud_scene, best_transform);
	ROS_INFO_STREAM("maximum number of inliers: " << max_inliers);

	// transform the point cloud from the ransac model and then use icp to minimize
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_scene);
	icp.setInputTarget(cloud_object);
	icp.setMaxCorrespondenceDistance(leaf*3.0f*0.8f);
	icp.setMaximumIterations(10);
	icp.setRANSACIterations(1000);
	icp.setRANSACOutlierRejectionThreshold(leaf*0.8f);
	icp.setTransformationEpsilon(1e-5);
	icp.setEuclideanFitnessEpsilon(1e-5);
	icp.align(*cloud_scene);
	Eigen::Matrix4f transformation_icp = icp.getFinalTransformation();
	transformPointCloud(*cloud_scene, *cloud_scene, transformation_icp);

	// icp convergence
	ROS_INFO_STREAM("converged - " << icp.hasConverged() << " - score - " << icp.getFitnessScore());

	// visualize only if flag is set
	if (vis)
	{
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_scene, 0.0, 255.0, 0.0), "scene");
		viewer->addPointCloud<pcl::PointXYZ>(cloud_object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_object, 255.0, 0.0, 0.0), "object");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object");
		viewer->addCoordinateSystem(0.1);
		viewer->initCameraParameters();

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			ros::Duration(0.01).sleep();
		}

		viewer->close();
	}

	Eigen::Isometry3f isof((transformation_icp * best_transform).inverse());
	return Eigen::Isometry3d(isof.matrix().cast<double>() * rhs_coord);
}

void
set_obj(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_object_)
{
	*cloud_object = *cloud_object_;

	// apply a voxel filter to the object
	{
		pcl::VoxelGrid<pcl::PointXYZ> voxel;
		voxel.setInputCloud(cloud_object);
		voxel.setLeafSize(leaf, leaf, leaf);
		voxel.filter(*cloud_object);
	}
}

std::tuple<int, Eigen::Matrix4f> RANSAC
(
	const std::vector<std::tuple<double, int, int>> & correspondences,
	pcl::common::UniformGenerator<int> * generator,
	const int start,
	const int end
)
{
	static Eigen::Matrix4f best_transform;
	int max_inliers = 0;

	for(auto i = start; i < end; i++)
	{

		std::vector<int> matches;
		std::vector<int> scene_indices, object_indices;

		for(int j = 0; matches.size() < 3;)
		{
			// generate a set of three correspondences
			auto number = generator->run();

			// ensure that it is three different correspondeces
			bool match = true;
			for(int k = 0; k < matches.size(); k++)
				match = (matches[k] == number) ? false : true;

			// if different correspondences continue
			if (not match)
				continue;

			// append to vector
			matches.push_back(number);
			scene_indices.push_back(std::get<1>(correspondences[matches[j]]));
			object_indices.push_back(std::get<2>(correspondences[matches[j]]));
			j += 1;
		}

		// ensure that the triangle equality holds, i.e., the area ratio should be somewhat 1.
		const auto object_area = abs((object_indices[0] - object_indices[1]) * (object_indices[0] - object_indices[2]));

		if (object_area < 1e-8)
			continue;

		const auto ratio_area = abs((scene_indices[0] - scene_indices[1]) * (scene_indices[0] - scene_indices[2])) / object_area;

		if (ratio_area > 1.2 or ratio_area < 0.8)
			continue;

		// estimate the rigid body transformations
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transform;
		Eigen::Matrix4f transformation;
		transform.estimateRigidTransformation(*cloud_scene, scene_indices, *cloud_object, object_indices, transformation);

		// transform scene
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scene_cloud (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*cloud_scene, *transformed_scene_cloud, transformation);

		// make a kdtree to check number of inliers
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(transformed_scene_cloud);

		// number of inliers
		int inliers = 0;

		// k=1 nearest neighbour search
		const int k = 1;
		std::vector<int> point_idx_knn_search(k);
		std::vector<float> point_knn_squared_distance(k);

		for(auto pt : cloud_object->points)
		{
			if (kdtree.nearestKSearch(pt, k, point_idx_knn_search, point_knn_squared_distance) > 0; point_knn_squared_distance[0] < std::pow(leaf, 2))
				inliers++;
		}

		if(max_inliers < inliers)
		{
			max_inliers = inliers;
			best_transform = transformation;
			ROS_INFO_STREAM("Found better match with " << inliers << " inliers!");
		}
	}

	return std::tuple<int, Eigen::Matrix4f>(max_inliers, best_transform);
}

}