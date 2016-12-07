/**
 * Author: rodrigo
 * 2016
 */
#include "GraspingUtils.hpp"
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include "Metric.hpp"
#include "PointFactory.hpp"


bool GraspingUtils::getTransformation(tf::StampedTransform &transform_,
									  const tf::TransformListener *tfListener_,
									  const std::string &target_,
									  const std::string &source_,
									  const ros::Time &time_,
									  const ros::Duration &timeout_)
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

geometry_msgs::Point GraspingUtils::transformPoint(const tf::TransformListener *tfListener_,
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

pcl::PointCloud<pcl::PointXYZ>::Ptr GraspingUtils::planeClipping(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
		const tf::StampedTransform &transformation,
		const ClippingAxis axis_,
		const float position_,
		const float normalOrientation_,
		pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud_)
{
	// Just to be sure the orientation factor is normalized
	float orientation = normalOrientation_ / fabs(normalOrientation_);

	// Prepare the clipping position and plane's normal
	tf::Vector3 tfPoint, tfNormal;
	switch (axis_)
	{
	default:
	case AXIS_X:
		tfPoint = transformation * tf::Vector3(position_, 0, 0);
		tfNormal = transformation.getBasis().getColumn(0) * orientation;
		break;

	case AXIS_Y:
		tfPoint = transformation * tf::Vector3(0, position_, 0);
		tfNormal = transformation.getBasis().getColumn(1) * orientation;
		break;

	case AXIS_Z:
		tfPoint = transformation * tf::Vector3(0, 0, position_);
		tfNormal = transformation.getBasis().getColumn(2) * orientation;
		break;
	}

	Eigen::Vector3f point = Eigen::Vector3f(tfPoint.x(), tfPoint.y(), tfPoint.z());
	Eigen::Vector3f normal = Eigen::Vector3f(tfNormal.x(), tfNormal.y(), tfNormal.z());

	Eigen::Hyperplane<float, 3> clippingPlane = Eigen::Hyperplane<float, 3>(normal, point);
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
		planeCloud_->clear();
		for (float i = -2; i <= 2; i += 0.05)
			for (float j = -2; j <= 2; j += 0.05)
			{
				Eigen::Vector3f p;
				switch (axis_)
				{
				default:
				case AXIS_X:
					p = Eigen::Vector3f(i, 0, j);
					break;

				case AXIS_Y:
					p = Eigen::Vector3f(0, i, j);
					break;

				case AXIS_Z:
					p = Eigen::Vector3f(i, j, 0);
					break;
				}
				planeCloud_->push_back(PointFactory::createPointXYZ(clippingPlane.projection(p)));
			}
	}

	return filteredCloud;
}

void GraspingUtils::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
									const float voxelSize_,
									pcl::PointCloud<pcl::PointXYZ>::Ptr &sampledCloud_)
{
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(cloud_);
	grid.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
	grid.filter(*sampledCloud_);
}

geometry_msgs::Pose GraspingUtils::genPose(const float x_,
		const float y_,
		const float z_,
		const float theta_,
		const float dirx_,
		const float diry_,
		const float dirz_)
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

int GraspingUtils::findNearestPoint(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
									const pcl::PointNormal &target_)
{
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdTree(new pcl::search::KdTree<pcl::PointNormal>);
	kdTree->setInputCloud(cloud_);

	std::vector<int> indices;
	std::vector<float> sqrDistances;
	kdTree->nearestKSearch(target_, 1, indices, sqrDistances);

	return indices[0];
}
