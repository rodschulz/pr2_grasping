import numpy as np
import sensor_msgs.point_cloud2 as pc

def extractLabeledCloud(data_):
	pts = 0
	clouds = {}

	for p in pc.read_points(data_, skip_nans=True, field_names=('x', 'y', 'z', 'label')):
		label = p[3]
		if not label in clouds:
			clouds[label] = []

		clouds[label].append([p[0], p[1], p[2]])
		pts = pts + 1

	return [clouds, pts]