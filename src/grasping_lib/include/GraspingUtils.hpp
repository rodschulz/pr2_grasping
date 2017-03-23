/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pr2_grasping/GraspingPoint.h>
#include <pr2_grasping/DescriptorCalc.h>


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


// Auxiliary structure to easy the grasping candidate generation process
struct CandidateData
{
	geometry_msgs::PoseStamped pose;
	pr2_grasping::DescriptorCalc::Response descriptor;
	pr2_grasping::GraspingPoint point;
	float score;
	float angle;
	size_t indexPoint;
	size_t indexAngle;

	CandidateData()
	{
		score = 0;
		angle = 0;
		indexPoint = 0;
		indexAngle = 0;
	}

	CandidateData(const CandidateData &other_)
	{
		pose = other_.pose;
		descriptor = other_.descriptor;
		point = other_.point;
		score = other_.score;
		angle = other_.angle;
		indexPoint = other_.indexPoint;
		indexAngle = other_.indexAngle;
	}
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
	static void generateGraspCloud(const std::string &filename_,
								   const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
								   const float searchRadius_,
								   const bool bidirectional_,
								   const geometry_msgs::Pose &target_,
								   const int nearest_,
								   const std::vector<Eigen::ParametrizedLine<float, 3> > &bandsAxes_);

	/**************************************************/
	static void generateGraspCloud(const std::string &filename_,
								   const CandidateData &candiate_);

private:
	// Constructor
	GraspingUtils();
	// Destructor
	~GraspingUtils();
};
