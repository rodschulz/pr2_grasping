#!/usr/bin/env python
import yaml
import utils
import rospy as rp
import numpy as np
from sklearn import metrics
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


from pr2_grasping.msg import GraspingPoint


# Debug flag 
debug = False
# Object for publishing the grasping points 
publisher = None
publisher2 = None


##################################################
def synthesizeGraspingPoints(points_, normals_, labels_, index_):
	points = []

	classes = set(labels_)
	for cls in classes:
		if cls == -1:
			continue

		pts = points_[labels_ == cls]
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
	clouds, normals, npts = utils.extractLabeledCloud(data_)
	rp.loginfo('Retrieved %d pts', npts)

	# Compute DBSCAN
	graspPoints = []
	minDataSize = 0.1 * npts
	for key in clouds:
		if (len(clouds[key]) < minDataSize) or (len(clouds[key]) < 10):
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
			normalsData = np.array(normals[key])
			graspPoints = graspPoints + synthesizeGraspingPoints(points_=cloudData, normals_=normalsData, labels_=db.labels_, index_=key)

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


	msg2 = GraspingPoint()
	msg2.header.frame_id = 'base_footprint'
	msg2.position.x = 1
	msg2.position.y = 2
	msg2.position.z = 3
	msg2.normal.x = 0.1
	msg2.normal.y = 0.2
	msg2.normal.z = 0.3
	publisher2.publish(msg2)




##################################################
if __name__ == '__main__':
	try:
		# Load config file
		with open(utils.getConfigPath(), 'r') as f:
			config = yaml.load(f)
			debug = config['analyzerDebug']

		# Initialize published topic
		publisher = rp.Publisher('/pr2_grasping/grasping_points', Float32MultiArray, queue_size=2)
		publisher2 = rp.Publisher('/pr2_grasping/grasping_points2', GraspingPoint, queue_size=10)

		# Setup node name
		rp.init_node('pr2_cluster_analyzer', anonymous=False)

		# Setup subscriber
		rp.Subscriber('/pr2_grasping/labeled_cloud', PointCloud2, analyze)

		# Spin until the node is stopped
		rp.spin()
	
	except IOError as e:
		print('Unable to read config file')

	except rp.ROSInterruptException as e:
		print('Node interrupted: ' + str(e))
		pass