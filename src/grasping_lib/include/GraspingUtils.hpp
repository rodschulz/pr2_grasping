/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#define PACKAGE_NAME		"pr2_grasping"
#define CONFIG_LOCATION		"config/config.yaml"


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
	static std::string getConfigPath();

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

private:
	// Constructor
	GraspingUtils();
	// Destructor
	~GraspingUtils();
};
