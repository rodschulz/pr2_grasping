/**
 * Author: rodrigo
 * 2016     
 */
#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include "Metric.hpp"
#include "PointFactory.hpp"

#define CONFIG_LOCATION		"src/grasping/config/config.yaml"
#define FRAME_KINNECT		"head_mount_kinect_ir_optical_frame"
#define FRAME_BASE			"base_link"

// Class implementing several utilities for the grasping node's routines
class GraspingUtils
{
public:
	// Gets the transformation between the given reference systems
	static inline bool getTransformation(tf::StampedTransform &transform_, const tf::TransformListener *transformationListener_, const std::string &target_, const std::string &source_, const ros::Time &time_ = ros::Time::now(), const ros::Duration &timeout_ = ros::Duration(1.0))
	{
		try
		{
			transformationListener_->waitForTransform(target_, source_, time_, timeout_);
			transformationListener_->lookupTransform(target_, source_, ros::Time(0), transform_);
			return true;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			return false;
		}
	}

	// Performs the extration of the interesting part of the cloud using an ad-hoc criterion
	static inline pcl::PointCloud<pcl::PointXYZ>::Ptr basicObjectExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_, const tf::StampedTransform &transformation, const bool debug_ = false)
	{
		tf::Vector3 point = transformation * tf::Vector3(0, 0, 0.77);

		// Calculate the clipping plane
		tf::Vector3 localNormal = transformation.getBasis().getColumn(2);
		Eigen::Vector3f globalNormal = Eigen::Vector3f(localNormal.x(), localNormal.y(), localNormal.z());
		Eigen::Vector3f globalPoint = Eigen::Vector3f(point.x(), point.y(), point.z());

		Eigen::Hyperplane<float, 3> clippingPlane = Eigen::Hyperplane<float, 3>(globalNormal, globalPoint);
		Eigen::Hyperplane<float, 3>::Coefficients planeCoeffs = clippingPlane.coeffs();

		if (debug_)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlane = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
			for (float i = 0; i < 2; i += 0.1)
				for (float j = 0; j < 2; j += 0.1)
					cloudPlane->push_back(PointFactory::createPointXYZ(clippingPlane.projection(Eigen::Vector3f(i, j, 0))));

			pcl::io::savePCDFileASCII("./trimming_plane.pcd", *cloudPlane);
		}

		// Get the filtered point indices
		std::vector<int> clippedPoints, idxs;
		pcl::PlaneClipper3D<pcl::PointXYZ> filter = pcl::PlaneClipper3D<pcl::PointXYZ>(planeCoeffs);
		filter.clipPointCloud3D(*cloud_, clippedPoints, idxs);

		// Generate the filtered cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		for (size_t i = 0; i < clippedPoints.size(); i++)
			filteredCloud->push_back(cloud_->at(clippedPoints[i]));

		return filteredCloud;
	}

	// Downsamples the given cloud
	static inline void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_, const float voxelSize_, pcl::PointCloud<pcl::PointXYZ>::Ptr &sampledCloud_)
	{
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setInputCloud(cloud_);
		grid.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
		grid.filter(*sampledCloud_);
	}

	static inline geometry_msgs::Pose genPose(const float x_, const float y_, const float z_, const float theta_ = 0, const float dirx_ = 1, const float diry_ = 0, const float dirz_ = 0)
	{
		geometry_msgs::Pose pose;

		pose.position.x = x_;
		pose.position.y = y_;
		pose.position.z = z_;

		Eigen::Vector3f p = Eigen::Vector3f(dirx_, diry_, dirz_).normalized();
		pose.orientation.w = cos(theta_ / 2);
		pose.orientation.x = p.x() * sin(theta_ / 2);
		pose.orientation.y = p.y() * sin(theta_ / 2);
		pose.orientation.z = p.z() * sin(theta_ / 2);

		return pose;
	}

private:
	// Constructor
	GraspingUtils();
	// Destructor
	~GraspingUtils();
};
