#!/usr/bin/env python
import yaml
import utils
import rospy as rp
import numpy as np
from sklearn import metrics
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
from pr2_grasping.msg import GraspingPoint
from pr2_grasping.msg import GraspingPointArray


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
def analyze(pointCloud_):
	rp.loginfo('Cloud received')

	# Extract data
	points, normals, npts = utils.extractLabeledCloud(pointCloud_)
	rp.loginfo('Retrieved %d pts', npts)

	# Compute DBSCAN
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
			# Calculate the silhouette coeff only if there's more than 1 cluster
			# if nclusters > 1:
			# 	rp.loginfo('.....silhouette: %0.3f', metrics.silhouette_score(positionData, db.labels_))

			# Synthesize the grasping points
			normalsData = np.array(normals[key])
			graspPts = graspPts + synthesizePoints(pointCloud_.header.frame_id, positionData, normalsData, db.labels_, key)

			# Generate debug data if requested
			if debug:
				utils.plotData3D(positionData, db.labels_, key, nclusters)

	rp.loginfo('...finished')


	# Publish the synthesized grasping points
	msg = GraspingPointArray()
	msg.data = graspPts
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
		publisher = rp.Publisher('/pr2_grasping/grasping_points', GraspingPointArray, queue_size=10)

		# Setup subscriber
		rp.Subscriber('/pr2_grasping/labeled_cloud', PointCloud2, analyze)

		# Spin until the node is stopped
		rp.spin()
	
	except IOError as e:
		print('Unable to read config file')

	except rp.ROSInterruptException as e:
		print('Node interrupted: ' + str(e))
		pass