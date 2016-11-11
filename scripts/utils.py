'''
@author: rodrigo
2016
'''
import numpy as np
import sensor_msgs.point_cloud2 as pc
import matplotlib.pyplot as plt
import matplotlib.figure as fig
from mpl_toolkits.mplot3d import Axes3D
import rospkg
import datetime


packageName = 'pr2_grasping'
configLocation = 'config/config.yaml'
outputLocation = 'output/'


##################################################
def getConfigPath():
	rospack = rospkg.RosPack()
	fullpath = rospack.get_path(packageName)
	return fullpath + '/' + configLocation


##################################################
def getOutputPath():
	rospack = rospkg.RosPack()
	fullpath = rospack.get_path(packageName)
	return fullpath + '/' + outputLocation


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
def getPalette(paletteName_):
	if paletteName_.lower() == 'spectral':
		return plt.cm.Spectral(np.linspace(0, 1, 5))

	elif paletteName_.lower() == 'set1':
		return plt.cm.Set1(np.linspace(0, 1, 9))

	elif paletteName_.lower() == 'set2':
		return plt.cm.Set2(np.linspace(0, 1, 7))

	elif paletteName_.lower() == 'set3':
		return plt.cm.Set3(np.linspace(0, 1, 12))

	else:
		return plt.cm.Set3(np.linspace(0, 1, 12))


##################################################
def plotData3D(path_, data_, labels_, index_=-1, nclusters_=-1, palette_ = ''):
	params = fig.SubplotParams(left=.02, right=.98, top=.99, bottom=.01)
	figure = plt.figure(figsize=(12, 9), subplotpars=params)
	ax = figure.add_subplot(111, projection='3d')

	classes = set(labels_)
	colors = getPalette(palette_)

	av = np.average(data_, axis=0)
	d = 0.15

	for cls, col in zip(classes, colors):
		if cls == -1:
			col = 'k'

		dclass = data_[labels_ == cls]
		xclass = extractByDimension(dclass, 0)
		yclass = extractByDimension(dclass, 1)
		zclass = extractByDimension(dclass, 2)


		ax.scatter3D(xclass, yclass, zclass, c=col, s=10, linewidth='0', alpha=1.0)
		title = 'label_' + str(index_)
		ax.set_title(title + ' (' + str(len(data_)) +' pts - ' + str(nclusters_) + ' clusters)')
		ax.view_init(elev=30, azim=-15)
		ax.set_xlabel('x')
		ax.set_ylabel('y')
		ax.set_zlabel('z') 
		ax.set_xlim3d(av[0] - d, av[0] + d)
		ax.set_ylim3d(av[1] - d, av[1] + d)
		ax.set_zlim3d(av[2] - d, av[2] + d)

	filename = path_ + title + '_{:%Y-%m-%d_%H%M%S}'.format(datetime.datetime.now()) + '.png'
	plt.savefig(filename)
