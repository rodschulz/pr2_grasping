#!/usr/bin/env python
import yaml
import utils
import rospy as rp
import numpy as np
from sklearn import metrics
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
from pr2_grasping.msg import GraspingData
from pr2_grasping.msg import GraspingPoint
from pr2_grasping.msg import GraspingPointArray
from pr2_grasping.msg import ObjectCloudData


# Debug flag 
debug = False
# Object for publishing the grasping points 
publisher = None


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
		normal = np.average(normals_[clusteringLabels_ == cls], axis=0)

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


		db = DBSCAN(eps=0.02, min_samples=20).fit(positionData)
		nclusters = len(set(db.labels_)) - (1 if -1 in db.labels_ else 0)
		rp.loginfo('...label %d (%d pts), found %d clusters', key, len(positionData), nclusters)


		if nclusters > 0:
			# Synthesize the grasping points
			normalsData = np.array(normals[key])
			graspPts = graspPts + synthesizePoints(data_.cloud.header.frame_id, positionData, normalsData, db.labels_, key)

			# Generate debug data if requested
			if debug:
				utils.plotData3D(positionData, db.labels_, key, nclusters)


	# Publish the synthesized grasping points
	rp.loginfo('...publishing %d grasping points', len(graspPts))

	if debug:
		for gp in graspPts:
			rp.loginfo('.....p=(%.3f, %.3f, %.3f) - n=(%.3f, %.3f, %.3f) - l=%d', gp.position.x, gp.position.y, gp.position.z, gp.normal.x, gp.normal.y, gp.normal.z, gp.label)

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

		# Setup node name
		rp.init_node('pr2_cluster_analyzer', anonymous=False)

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