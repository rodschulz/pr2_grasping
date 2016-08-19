#!/usr/bin/env python
import yaml
import utils
import rospy as rp
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN
from sklearn import metrics


# Node configuration file location
configLocation = 'src/grasping/config/config.yaml'
# Debug flag 
debug = False
# Object for publishing the grasping points 
publisher = None


##################################################
def synthesizeGraspingPoints(data_, labels_, index_):
	points = []

	classes = set(labels_)
	for cls in classes:
		if cls == -1:
			continue

		pts = data_[labels_ == cls]
		if len(pts) < 20 :
			continue

		rp.loginfo('.......cluster size: %d pts', len(pts))
		p = np.average(pts, axis=0)

		# Concatenate the new grasping point and its label
		points = points + p.tolist()
		points.append(index_)

	return points


##################################################
def analyze(data_):
	rp.loginfo('Cloud received')

	# Extract data
	clouds, npoints = utils.extractLabeledCloud(data_)
	rp.loginfo('Retrieved %d pts', npoints)

	# Compute DBSCAN
	graspPoints = []
	minDataSize = 0.1 * npoints
	for key in clouds:
		if len(clouds[key]) < minDataSize:
			continue

		# Get the actual data from the cloud
		cloudData = np.array(clouds[key])

		db = DBSCAN(eps=0.02, min_samples=20).fit(cloudData)
		nclusters = len(set(db.labels_)) - (1 if -1 in db.labels_ else 0)
		rp.loginfo('...label %d (%d pts), found %d clusters', key, len(cloudData), nclusters)

		if nclusters > 0:
			# Calculate the silhouette coeff only if there's more than 1 cluster
			if nclusters > 1:
				rp.loginfo('.....silhouette: %0.3f', metrics.silhouette_score(cloudData, db.labels_))

			# Synthesize the grasping points
			graspPoints = graspPoints + synthesizeGraspingPoints(data_=cloudData, labels_=db.labels_, index_=key)

			# Generate debug data if requested
			if debug:
				utils.plotData3D(data_=cloudData, labels_=db.labels_, index_=key, nclusters_=nclusters)

	rp.loginfo('...finished')

	# Publish the synthesized grasping points
	dims = 4
	npts = len(graspPoints) / dims;
	msg = Float32MultiArray()
	msg.layout.dim.append(MultiArrayDimension('len', npts, dims))
	msg.layout.dim.append(MultiArrayDimension('coords', dims, 1))
	msg.data = graspPoints
	publisher.publish(msg)


##################################################
if __name__ == '__main__':
	try:
		# Load config file
		with open(configLocation, 'r') as f:
			config = yaml.load(f)
			debug = config['analyzerDebug']

		# Initialize published topic
		publisher = rp.Publisher('/pr2_grasping/grasping_points', Float32MultiArray, queue_size=2)

		# Setup node name
		rp.init_node('pr2_cluster_analizer', anonymous=False)

		# Setup subscriber
		rp.Subscriber('/pr2_grasping/labeled_cloud', PointCloud2, analyze)

		# Spin until the node is stopped
		rp.spin()
	
	except IOError as e:
		print('Unable to read config file')

	except rp.ROSInterruptException as e:
		print('Node interrupted: ' + str(e))
		pass