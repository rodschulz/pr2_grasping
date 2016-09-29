/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include "Metric.hpp"
#include "PointFactory.hpp"


#define PACKAGE_NAME		"pr2_grasping"
#define CONFIG_LOCATION		"config/config.yaml"


// Class implementing several utilities for the grasping node's routines
class GraspingUtils
{
public:
	// Returns the full path of the package's configuration file
	static inline std::string getConfigPath()
	{
		std::string packagePath = ros::package::getPath(PACKAGE_NAME);
		std::string fullpath = packagePath + "/" + CONFIG_LOCATION;
		return fullpath;
	}


	// Gets the transformation between the given reference systems
	static inline bool getTransformation(tf::StampedTransform &transform_,
										 const tf::TransformListener *tfListener_,
										 const std::string &target_,
										 const std::string &source_,
										 const ros::Time &time_ = ros::Time::now(),
										 const ros::Duration &timeout_ = ros::Duration(1.0))
	{
		try
		{
			tfListener_->waitForTransform(target_, source_, time_, timeout_);
			tfListener_->lookupTransform(target_, source_, ros::Time(0), transform_);
			return true;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			return false;
		}
	}


	// Transforms a pose between the given source and target reference frames
	static inline geometry_msgs::Point transformPoint(const tf::TransformListener *tfListener_,
			const std::string &target_,
			const std::string &source_,
			const geometry_msgs::Point &point_)
	{
		tf::StampedTransform toTarget;
		while (!GraspingUtils::getTransformation(toTarget, tfListener_, target_, source_));
		tf::Vector3 conv = toTarget * tf::Vector3(point_.x, point_.y, point_.z);

		geometry_msgs::Point result;
		result.x = conv.x();
		result.y = conv.y();
		result.z = conv.z();
		return result;
	}


	// Clips the given cloud using a plane at the given Z
	static inline pcl::PointCloud<pcl::PointXYZ>::Ptr basicPlaneClippingZ(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
			const tf::StampedTransform &transformation,
			const float clippingZ_,
			pcl::PointCloud<pcl::PointXYZ>::Ptr &planeCloud_)
	{
		tf::Vector3 point = transformation * tf::Vector3(0, 0, clippingZ_);

		// Calculate the clipping plane
		tf::Vector3 localNormal = transformation.getBasis().getColumn(2);
		Eigen::Vector3f globalNormal = Eigen::Vector3f(localNormal.x(), localNormal.y(), localNormal.z());
		Eigen::Vector3f globalPoint = Eigen::Vector3f(point.x(), point.y(), point.z());

		Eigen::Hyperplane<float, 3> clippingPlane = Eigen::Hyperplane<float, 3>(globalNormal, globalPoint);
		Eigen::Hyperplane<float, 3>::Coefficients planeCoeffs = clippingPlane.coeffs();

		// Get the filtered point indices
		std::vector<int> clippedPoints, idxs;
		pcl::PlaneClipper3D<pcl::PointXYZ> filter = pcl::PlaneClipper3D<pcl::PointXYZ>(planeCoeffs);
		filter.clipPointCloud3D(*cloud_, clippedPoints, idxs);

		// Generate the filtered cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		for (size_t i = 0; i < clippedPoints.size(); i++)
			filteredCloud->push_back(cloud_->at(clippedPoints[i]));

		// Generate a sample of the clipping plane output cloud if required
		if (planeCloud_.get() != NULL)
		{
			planeCloud_.clear();
			for (float i = -2; i < 2; i += 0.05)
				for (float j = -2; j < 2; j += 0.05)
					planeCloud_->push_back(PointFactory::createPointXYZ(clippingPlane.projection(Eigen::Vector3f(i, j, 0))));
		}

		return filteredCloud;
	}


	// Downsamples the given cloud
	static inline void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
									   const float voxelSize_,
									   pcl::PointCloud<pcl::PointXYZ>::Ptr &sampledCloud_)
	{
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setInputCloud(cloud_);
		grid.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
		grid.filter(*sampledCloud_);
	}


	// Generates a pose structure with the given data
	static inline geometry_msgs::Pose genPose(const float x_,
			const float y_,
			const float z_,
			const float theta_ = 0,
			const float dirx_ = 1,
			const float diry_ = 0,
			const float dirz_ = 0)
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


	// Generates a posture for the gripper's joints
	static inline trajectory_msgs::JointTrajectory generateGraspPosture(const float value_,
			const std::string gripperGroup_)
	{
		trajectory_msgs::JointTrajectory posture;
		moveit::planning_interface::MoveGroup gripper(gripperGroup_);

		// Set the active joints names
		std::vector<std::string> activeJoints = gripper.getActiveJoints();
		BOOST_FOREACH(const std::string name, activeJoints)
		{
			posture.joint_names.push_back(name);
		}

		posture.points.resize(1);
		posture.points[0].positions.resize(posture.joint_names.size(), value_);
		posture.points[0].time_from_start = ros::Duration(45.0);
		posture.points[0].effort.resize(posture.joint_names.size(), -1.0);

		return posture;
	}

private:
	// Constructor
	GraspingUtils();
	// Destructor
	~GraspingUtils();
};
