#!/usr/bin/env python
'''
@author: rodrigo
2016
'''
import yaml
import utils
import rospy as rp
import numpy as np
from sklearn import metrics
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
from pr2_grasping.msg import GraspingData
from pr2_grasping.msg import GraspingPoint
from pr2_grasping.msg import ObjectCloudData


##### Global variables #####
publisher = None
epsilon = 0.01
minPoints = 10
palette = 'Set1'

##### Debug variables flag #####
debug = False


##################################################
def filterByStd(data):
	av = np.average(data, axis=0)
	std = np.std(data, axis=0)

	filtered = None
	for d in data:
		dx = abs(d[0] - av[0])
		dy = abs(d[1] - av[1])
		if dx <= std[0] and dy <= std[1]:
			if filtered == None:
				filtered = np.array([d])
			else:
				filtered = np.concatenate((filtered, [d]), axis=0)

	return filtered


##################################################
def synthesizePoints(frameId_, points_, normals_, clusteringLabels_, index_):
	points = []

	classes = set(clusteringLabels_)
	for cls in classes:
		if cls == -1:
			continue

		pts = points_[clusteringLabels_ == cls]
		if len(pts) < 20 :
			continue

		rp.loginfo('.......cluster size: %d pts', len(pts))

		position = np.average(pts, axis=0)
		# normal = np.average(normals_[clusteringLabels_ == cls], axis=0)
		filteredNormals = filterByStd(normals_[clusteringLabels_ == cls])
		normal = np.average(filteredNormals, axis=0)

		point = GraspingPoint()
		point.header.frame_id = frameId_
		point.label = index_
		point.position.x = position[0]
		point.position.y = position[1]
		point.position.z = position[2]
		point.normal.x = normal[0]
		point.normal.y = normal[1]
		point.normal.z = normal[2]

		points.append(point)

	return points


##################################################
def analyze(data_):
	rp.loginfo('Cloud received')

	# Extract data
	points, normals, npts = utils.extractLabeledCloud(data_.cloud)
	rp.loginfo('Retrieved %d pts', npts)

	# Compute DBSCAN
	rp.loginfo('...synthesizing grasping points')
	graspPts = []
	minDataSize = 0.1 * npts
	for key in points:
		if (len(points[key]) < minDataSize) or (len(points[key]) < 10):
			continue

		# Get the actual data from the cloud
		positionData = np.array(points[key])


		db = DBSCAN(eps=epsilon, min_samples=minPoints).fit(positionData)
		nclusters = len(set(db.labels_)) - (1 if -1 in db.labels_ else 0)
		rp.loginfo('...label %d (%d pts), found %d clusters', key, len(positionData), nclusters)


		if nclusters > 0:
			# Synthesize the grasping points
			normalsData = np.array(normals[key])
			graspPts = graspPts + synthesizePoints(data_.cloud.header.frame_id, positionData, normalsData, db.labels_, key)

			# Generate debug data if requested
			if debug:
				utils.plotData3D(utils.getOutputPath(), positionData, db.labels_, key, nclusters, palette)


	# Publish the synthesized grasping points
	rp.loginfo('...publishing %d grasping points', len(graspPts))

	if debug:
		for gp in graspPts:
			rp.logdebug('.....p=(%.3f, %.3f, %.3f) - n=(%.3f, %.3f, %.3f) - l=%d', gp.position.x, gp.position.y, gp.position.z, gp.normal.x, gp.normal.y, gp.normal.z, gp.label)

	msg = GraspingData()
	msg.graspingPoints = graspPts
	msg.boundingBoxMin = data_.boundingBoxMin
	msg.boundingBoxMax = data_.boundingBoxMax
	publisher.publish(msg)


##################################################
if __name__ == '__main__':
	try:
		# Load config file
		with open(utils.getConfigPath(), 'r') as f:
			config = yaml.load(f)
			debug = config['analyzerDebug']
			epsilon = config['analyzer']['epsilon']
			minPoints = config['analyzer']['minPoints']
			palette = config['analyzer']['palette']

		# Setup node name
		level = rp.INFO
		if debug:
			level = rp.DEBUG
		rp.init_node('pr2_cluster_analyzer', anonymous=False, log_level=level)

		# Initialize published topic
		publisher = rp.Publisher('/pr2_grasping/grasping_data', GraspingData, queue_size=10)

		# Setup subscriber
		rp.Subscriber('/pr2_grasping/object_cloud_data', ObjectCloudData, analyze)

		# Spin until the node is stopped
		rp.loginfo("Analyzer node spinning")
		rp.spin()
	
	except IOError as e:
		print('Unable to read config file')

	except rp.ROSInterruptException as e:
		print('Node interrupted: ' + str(e))
		pass