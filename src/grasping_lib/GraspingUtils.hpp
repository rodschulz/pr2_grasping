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

// SVM's shared pointer
//typedef boost::shared_ptr<CvSVM> CvSVMPtr;

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

	// Generates a permutation of the given matrix
	static inline void generatePermutation(const cv::Mat &matrix_, const int permutationSize_, const int permutationNumber_, cv::Mat &permutation_)
	{
		int begin = permutationNumber_ * permutationSize_;
		int end = matrix_.cols;
		matrix_.colRange(begin, end).copyTo(permutation_.colRange(0, end - begin));

		begin = 0;
		end = permutationNumber_ * permutationSize_;
		if (end - begin > 0)
			matrix_.colRange(begin, end).copyTo(permutation_.colRange(permutation_.cols - (end - begin), permutation_.cols));
	}

	// Prepares the clasificator for the labeling process
//	static CvSVMPtr prepareClasificator(const cv::Mat &bow_, const std::map<std::string, std::string> &bowParams_)
//	{
//		std::vector<std::string> params;
//		std::string metricParams = bowParams_.at("metric");
//		boost::trim_if(metricParams, boost::is_any_of("[]"));
//		boost::split(params, metricParams, boost::is_any_of("[,]"), boost::token_compress_on);
//
//		int permutationSize = bow_.cols;
//		if (boost::iequals(params[0], metricType[METRIC_CLOSEST_PERMUTATION]) || boost::iequals(params[0], metricType[METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE]))
//			permutationSize = boost::lexical_cast<int>(params[1]);
//
//		int permutationNumber = bow_.cols / permutationSize;
//
//		// Prepare the training data
//		cv::Mat trainingData = cv::Mat::zeros(bow_.rows * permutationNumber, bow_.cols, CV_32FC1);
//		cv::Mat labels = cv::Mat::zeros(bow_.rows * permutationNumber, 1, CV_32FC1);
//
//		for (int i = 0; i < bow_.rows; i++)
//		{
//			cv::Mat currentCentroid = bow_.row(i);
//			for (int j = 0; j < permutationNumber; j++)
//			{
//				int index = i * permutationNumber + j;
//
//				cv::Mat permutation = trainingData.row(index);
//				generatePermutation(currentCentroid, permutationSize, j, permutation);
//				labels.at<float>(index, 0) = (float) (i + 1);
//			}
//		}
//
//		// Set SVM parameters
//		CvSVMParams svmParams;
//		svmParams.svm_type = CvSVM::C_SVC;
//		svmParams.kernel_type = CvSVM::LINEAR;
//		svmParams.term_crit = cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-6);
//
//		// Train the SVM
//		CvSVMPtr svm = CvSVMPtr(new CvSVM());
//		svm->train(trainingData, labels, cv::Mat(), cv::Mat(), svmParams);
//
//		return svm;
//	}

	// Downsamples the given cloud
	static inline void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_, const float voxelSize_, pcl::PointCloud<pcl::PointXYZ>::Ptr &sampledCloud_)
	{
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setInputCloud(cloud_);
		grid.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
		grid.filter(*sampledCloud_);
	}

private:
	// Constructor
	GraspingUtils();
	// Destructor
	~GraspingUtils();
};
