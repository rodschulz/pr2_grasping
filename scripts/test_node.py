#!/usr/bin/env python
import yaml
import utils
import rospy as rp
import numpy as np
import numpy.linalg as la
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


padding = 0.2

##################################################
def genPose(x_, y_, z_, ax_, ay_, az_, theta_):
	pose = Pose()
	pose.position.x = x_
	pose.position.y = y_
	pose.position.z = z_

	v = np.array([ax_, ay_, az_])
	v = v / la.norm(v)
	
	angle = theta_ / 2.0
	pose.orientation.w = np.cos(angle)
	pose.orientation.x = v[0] * np.sin(angle)
	pose.orientation.y = v[1] * np.sin(angle)
	pose.orientation.z = v[2] * np.sin(angle)

	return pose


def genPose2(x_, y_, z_, ax_, ay_, az_):
	pose = Pose()

	pose.position.x = x
	pose.position.y = y
	pose.position.z = z
	
	v = np.array([nx, ny, nz])
	v = v / la.norm(v)
	
	pose.orientation.w = 0
	pose.orientation.x = v[0]
	pose.orientation.y = v[1]
	pose.orientation.z = v[2]

	return pose


def calcGraspPose(x_, y_, z_, nx_, ny_, nz_):
	v = np.array([nx_, ny_, nz_])
	v = v / la.norm(v)

	p = np.array([x_, y_, z_])
	gp = p + padding * v

	pose = Pose()
	pose.position.x = gp[0]
	pose.position.y = gp[1]
	pose.position.z = gp[2]
	pose.orientation.w = 0
	pose.orientation.x = v[0]
	pose.orientation.y = v[1]
	pose.orientation.z = v[2]

	return pose


def calcGraspPose2(x_, y_, z_, nx_, ny_, nz_):
	v = np.array([nx_, ny_, nz_])
	v = v / la.norm(v)

	p = np.array([x_, y_, z_])
	gp = p + padding * v

	pose = Pose()
	pose.position.x = gp[0]
	pose.position.y = gp[1]
	pose.position.z = gp[2]

	nn = p - gp
	pose.orientation.w = 0
	pose.orientation.x = nn[0]
	pose.orientation.y = nn[1]
	pose.orientation.z = nn[2]

	return pose

##################################################
if __name__ == '__main__':
	try:
		# Setup node name
		rp.init_node('test_node', anonymous=False)

		# Initialize published topic
		graspingPointPub = rp.Publisher('/pr2_grasping/debug_grasping_point', PoseStamped, queue_size=10)
		graspingPosePub = rp.Publisher('/pr2_grasping/grasping_pose', PoseStamped, queue_size=10)


		# x = 0.722
		# y = 0.010
		# z = 0.784
		# nx = -0.762
		# ny = 0.006
		# nz = -0.111

		x = 0.727
		y = -0.002
		z = 0.724
		nx = -0.600
		ny = -0.333
		nz = -0.000

		# x = 0.730
		# y = 0.019
		# z = 0.831
		# nx = -0.385
		# ny = 0.296
		# nz = -0.222

		graspPoint = PoseStamped()
		graspPoint.header.frame_id = 'base_footprint';
		graspPoint.pose = genPose2(x, y, z, nx, ny, nz)
		# pose.pose = genPose(0.732, 0.006, 0.798, np.deg2rad(270), 0, 1, 0);


		graspPose = PoseStamped()
		graspPose.header.frame_id = 'base_footprint'
		graspPose.pose = calcGraspPose(x, y, z, nx, ny, nz)
		# graspPose.pose = calcGraspPose2(x, y, z, nx, ny, nz)


		while True:
			graspingPointPub.publish(graspPoint)
			graspingPosePub.publish(graspPose)

			rp.loginfo("Publishing")
			rp.sleep(0.2)


		# Spin until the node is stopped
		rp.loginfo("Test spinning")
		rp.spin()

	except rp.ROSInterruptException as e:
		print('Node interrupted: ' + str(e))
		pass