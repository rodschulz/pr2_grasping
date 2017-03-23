/**
 * Author: rodrigo
 * 2016
 */
#include "GraspingUtils.hpp"
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Metric.hpp"
#include "PointFactory.hpp"
#include "CloudFactory.hpp"
#include "PkgUtils.hpp"
#include "Extractor.hpp"
#include "Config.hpp"


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
									const geometry_msgs::Pose &target_)
{
	// Add the grasping point to the cloud
	float x = target_.position.x;
	float y = target_.position.y;
	float z = target_.position.z;

	float dx = target_.orientation.x;
	float dy = target_.orientation.y;
	float dz = target_.orientation.z;
	float dw = target_.orientation.w;

	Eigen::Vector3f origin = Eigen::Vector3f(x, y, z);
	Eigen::Vector3f direction = Eigen::Quaternionf(dw, dx, dy, dz) * Eigen::Vector3f(1, 0, 0);
	Eigen::ParametrizedLine<float, 3> line(origin, direction);


	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud_);

	int target = -1;
	float distance = std::numeric_limits<float>::max();
	float step = 0.001;
	for (float t = step; t < 0.2; t += step)
	{
		Eigen::Vector3f p = line.pointAt(t);
		std::vector<int> indices;
		std::vector<float> distances;
		kdtree.nearestKSearch(PointFactory::createPointNormal(p.x(), p.y(), p.z(), 0, 0, 0), 1, indices, distances);

		if (target == -1 || distances[0] < distance)
		{
			target = indices[0];
			distance = distances[0];
		}
	}

	return target;
}

void GraspingUtils::generateGraspCloud(const std::string &filename_,
									   const CandidateData &candiate_)
{
	ROS_DEBUG("Transforming cloud");
	// Transform cloud type
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
	pcl::fromROSMsg(candiate_.descriptor.cloud, *cloud);


	ROS_DEBUG("Producing axes");
	// Generate band axes
	std::vector<Eigen::ParametrizedLine<float, 3> > axes;
	for (size_t band = 0; band < candiate_.descriptor.bandAxes.size(); band++)
	{
		geometry_msgs::Point o = candiate_.descriptor.bandAxes[band].origin;
		geometry_msgs::Point d = candiate_.descriptor.bandAxes[band].director;
		axes.push_back(Eigen::ParametrizedLine<float, 3>(Eigen::Vector3f(o.x, o.y, o.z), Eigen::Vector3f(d.x, d.y, d.z).normalized()));
	}


	ROS_DEBUG("Calling method");
	// Generate the cloud
	GraspingUtils::generateGraspCloud(filename_,
									  cloud,
									  candiate_.descriptor.params.searchRadius,
									  candiate_.descriptor.params.bidirectional,
									  candiate_.pose.pose,
									  candiate_.descriptor.index.data,
									  axes);
}


void GraspingUtils::generateGraspCloud(const std::string &filename_,
									   const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
									   const float searchRadius_,
									   const bool bidirectional_,
									   const geometry_msgs::Pose &target_,
									   const int nearest_,
									   const std::vector<Eigen::ParametrizedLine<float, 3> > &bandsAxes_)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr grasp = CloudFactory::createColorCloud(cloud_, Utils::palette12(0));


	// Draw the vicinity used for computation
	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(cloud_, cloud_->at(nearest_), searchRadius_);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr patchCloud = CloudFactory::createColorCloud(patch, COLOR_SLATE_GRAY);
	*grasp += *patchCloud;


	// Draw the band axes
	for (size_t b = 0; b < bandsAxes_.size(); b++)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr lineCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
		for (float t = (bidirectional_ ? -searchRadius_ : 0);
				t <= searchRadius_;
				t += searchRadius_ / 50)
		{
			Eigen::Vector3f point = bandsAxes_[b].pointAt(t);
			lineCloud->push_back(PointFactory::createPointXYZRGBNormal(point.x(), point.y(), point.z(), 0, 0, 0, 0, (PointColor)Utils::palette12(b + 1)));
		}

		*grasp += *lineCloud;
	}


	float x = target_.position.x;
	float y = target_.position.y;
	float z = target_.position.z;

	float dx = target_.orientation.x;
	float dy = target_.orientation.y;
	float dz = target_.orientation.z;
	float dw = target_.orientation.w;

	Eigen::Vector3f origin = Eigen::Vector3f(x, y, z);
	Eigen::Vector3f direction = Eigen::Quaternionf(dw, dx, dy, dz) * Eigen::Vector3f(1, 0, 0);
	Eigen::ParametrizedLine<float, 3> line(origin, direction);


	// Draw a line showing the approach vector
	float delta = 0.2;
	float step = 0.001;
	for (float t = step; t < delta; t += step)
	{
		Eigen::Vector3f p = line.pointAt(t);
		grasp->push_back(PointFactory::createPointXYZRGBNormal(p.x(), p.y(), p.z(), 0, 0, 0, 0, COLOR_LIME));
	}


	// Draw the nearest point
	float xnearest = grasp->at(nearest_).x;
	float ynearest = grasp->at(nearest_).y;
	float znearest = grasp->at(nearest_).z;
	grasp->push_back(PointFactory::createPointXYZRGBNormal(xnearest, ynearest, znearest, 0, 0, 0, 0, COLOR_YELLOW));


	// Draw the target point
	grasp->push_back(PointFactory::createPointXYZRGBNormal(x, y, z, 0, 0, 0, 0, COLOR_RED));


	std::string outputDir = PkgUtils::getOutputPath();
	pcl::io::savePCDFileASCII(outputDir + DEBUG_PREFIX + filename_ + CLOUD_FILE_EXTENSION, *grasp);
}