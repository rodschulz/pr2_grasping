#!/usr/bin/env python
import rospy as rp
import numpy as np
import yaml
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN
from sklearn import metrics
from mpl_toolkits.mplot3d import Axes3D

configLocation = 'src/grasping/config/config_labeler.yaml'
debug = False

##################################################
def extractDim(data, dim=0):
	extracted = []
	for d in data:
		extracted.append(d[dim])
	return extracted

##################################################
def plotData(data, labels):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	d = np.array(data)
	classes = set(labels)
	colors = plt.cm.Spectral(np.linspace(0, 1, len(classes)))

	for k, col in zip(classes, colors):
		if k == -1:
			col = 'k'

		mask = (labels == k)
		dclass = d[mask]
		xclass = extractDim(dclass, 0)
		yclass = extractDim(dclass, 1)
		zclass = extractDim(dclass, 2)
		ax.scatter(xclass, yclass, zclass, c=col, s=20, linewidth='0', alpha=1.0)

	plt.show()

##################################################
def analyze(data):
	# Extract data
	rp.loginfo('Extracting data')

	k = 0
	clouds = {}
	for p in pc.read_points(data, skip_nans=True, field_names=('x', 'y', 'z', 'label')):
		label = p[3]
		if not label in clouds:
			clouds[label] = []

		clouds[label].append([p[0], p[1], p[2]])
		k = k + 1
	rp.loginfo('Retrieved %d pts', k)

	# Compute DBSCAN
	for key in clouds:
		db = DBSCAN(eps=0.01, min_samples=10).fit(clouds[key])
		labels = db.labels_
		nclusters = len(set(labels)) - (1 if -1 in labels else 0)
		rp.loginfo('...label %d (%d pts), found %d clusters', key, len(clouds[key]), nclusters)

		if debug and nclusters > 0:
			plotData(data=clouds[key], labels=labels)

	rp.loginfo('Done')

##################################################
if __name__ == '__main__':
	try:
		with open(configLocation, 'r') as f:
			config = yaml.load(f)
			debug = config['analyzerDebug']

		# Setup node name
		rp.init_node('pr2_cluster_analizer', anonymous=False)

		# Setup subscriber
		rp.Subscriber('/pr2_grasping/labeled_cloud', PointCloud2, analyze)

		# Spin until the node is stopped
		rp.spin()
	
	except IOError as e:
		print('Unable to read config file')

	except rp.ROSInterruptException as e:
		pass