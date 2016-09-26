import numpy as np
import sensor_msgs.point_cloud2 as pc
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospkg


# Package's name
packageName = 'pr2_grasping'
# Relative location of node's config file 
configLocation = 'config/config.yaml'


##################################################
def getConfigPath():
	rospack = rospkg.RosPack()
	fullpath = rospack.get_path(packageName)
	return fullpath + '/' + configLocation


##################################################
def extractLabeledCloud(pointCloud_):
	npts = 0
	points = {}
	normals = {}

	xIdx = 0
	yIdx = 1
	zIdx = 2
	nxIdx = 3
	nyIdx = 4
	nzIdx = 5
	labelIdx = 6

	for p in pc.read_points(pointCloud_, skip_nans=True, field_names=('x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z', 'label')):
		label = p[labelIdx]

		if not label in points:
			points[label] = []
			normals[label] = []

		points[label].append([p[xIdx], p[yIdx], p[zIdx]])
		normals[label].append([p[nxIdx], p[nyIdx], p[nzIdx]])

		npts = npts + 1

	return [points, normals, npts]


##################################################
def extractByDimension(data_, dim_=0):
	extracted = []
	for d in data_:
		extracted.append(d[dim_])
	return extracted


##################################################
def plotData3D(data_, labels_, index_=-1, nclusters_=-1):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	classes = set(labels_)
	colors = plt.cm.Spectral(np.linspace(0, 1, len(classes)))

	for cls, col in zip(classes, colors):
		if cls == -1:
			col = 'k'

		dclass = data_[labels_ == cls]
		xclass = extractByDimension(dclass, 0)
		yclass = extractByDimension(dclass, 1)
		zclass = extractByDimension(dclass, 2)
		ax.scatter(xclass, yclass, zclass, c=col, s=20, linewidth='0', alpha=1.0)
		title = 'label' + str(index_)
		ax.set_title(title + ' (' + str(len(data_)) +' pts - ' + str(nclusters_) + ' clusters)')
		ax.view_init(elev=30, azim=-15)

	plt.savefig(title + '.png')
	# plt.show()