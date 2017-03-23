/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "DescriptorParams.hpp"
#include "Band.hpp"


#define OBJECT_TARGET		"object_target"
#define OBJECT_SUPPORT		"object_support"
#define GRASP_ID			"grasp_"


// Axis definition for the cloud clipping methods
enum ClippingAxis
{
	AXIS_X,
	AXIS_Y,
	AXIS_Z
};


// Class implementing several utilities for the grasping node's routines
class GraspingUtils
{
public:
	/**************************************************/
	static bool getTransformation(tf::StampedTransform &transform_,
								  const tf::TransformListener *tfListener_,
								  const std::string &target_,
								  const std::string &source_,
								  const ros::Time &time_ = ros::Time::now(),
								  const ros::Duration &timeout_ = ros::Duration(1.0));

	/**************************************************/
	static geometry_msgs::Point transformPoint(const tf::TransformListener *tfListener_,
			const std::string &target_,
			const std::string &source_,
			const geometry_msgs::Point &point_);

	/**************************************************/
	static pcl::PointCloud<pcl::PointXYZ>::Ptr planeClipping(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
			const tf::StampedTransform &transformation,
			const ClippingAxis axis_,
			const float position_,
			const float normalOrientation_,
			pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr());

	/**************************************************/
	static void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
								const float voxelSize_,
								pcl::PointCloud<pcl::PointXYZ>::Ptr &sampledCloud_);

	/**************************************************/
	static geometry_msgs::Pose genPose(const float x_,
									   const float y_,
									   const float z_,
									   const float theta_ = 0,
									   const float dirx_ = 1,
									   const float diry_ = 0,
									   const float dirz_ = 0);

	/**************************************************/
	static int findNearestPoint(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
								const geometry_msgs::Pose &target_);

	/**************************************************/
	static void generateGraspCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
								   const DCHParams *dchParams_,
								   const geometry_msgs::Pose &target_,
								   const int nearest_,
								   const std::vector<BandPtr> &bands_);

private:
	// Constructor
	GraspingUtils();
	// Destructor
	~GraspingUtils();
};
