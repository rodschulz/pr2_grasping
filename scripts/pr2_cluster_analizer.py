#!/usr/bin/env python
import yaml
import utils
import rospy as rp
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN
from sklearn import metrics
from mpl_toolkits.mplot3d import Axes3D

configLocation = 'src/grasping/config/config.yaml'
debug = False


##################################################
def extractDim(data_, dim_=0):
	extracted = []
	for d in data_:
		extracted.append(d[dim_])
	return extracted


##################################################
def plotData(data_, labels_, index_=-1, nclusters_=-1):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	classes = set(labels_)
	colors = plt.cm.Spectral(np.linspace(0, 1, len(classes)))

	for cls, col in zip(classes, colors):
		if cls == -1:
			col = 'k'

		dclass = data_[labels_ == cls]
		xclass = extractDim(dclass, 0)
		yclass = extractDim(dclass, 1)
		zclass = extractDim(dclass, 2)
		ax.scatter(xclass, yclass, zclass, c=col, s=20, linewidth='0', alpha=1.0)
		title = 'label' + str(index_)
		ax.set_title(title + ' (' + str(len(data_)) +' pts - ' + str(nclusters_) + ' clusters)')
		ax.view_init(elev=30, azim=-15)

	plt.savefig(title + '.png')
	# plt.show()


##################################################
def synthetizeGraspingPoint(data_, labels_, index_):
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
		points.append([p, index_])

	return points


##################################################
def analyze(data_):
	rp.loginfo('Cloud received')

	# Extract data
	clouds, pts = utils.extractLabeledCloud(data_)
	rp.loginfo('Retrieved %d pts', pts)

	# Compute DBSCAN
	minDataSize = 0.1 * pts
	for key in clouds:
		if len(clouds[key]) < minDataSize:
			continue

		points = np.array(clouds[key])

		db = DBSCAN(eps=0.02, min_samples=20).fit(points)
		nclusters = len(set(db.labels_)) - (1 if -1 in db.labels_ else 0)
		rp.loginfo('...label %d (%d pts), found %d clusters', key, len(points), nclusters)
		rp.loginfo('.....silhouette: %0.3f', metrics.silhouette_score(points, db.labels_))

		if nclusters > 0:
			graspPts = synthetizeGraspingPoint(data_=points, labels_=db.labels_, index_=key)

			# Generate debug data if requested
			if debug:
				plotData(data_=points, labels_=db.labels_, index_=key, nclusters_=nclusters)

	rp.loginfo('...finished')


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