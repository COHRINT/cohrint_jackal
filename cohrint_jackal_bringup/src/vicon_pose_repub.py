#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import numpy as np 



ASSET_NAME = "CASE1_1"

class ViconRepub:
	def __init__(self):
		self.pub = rospy.Publisher("/vicon_pose",PoseWithCovarianceStamped,queue_size=10)

		rospy.Subscriber("/vrpn_client_node/"+ASSET_NAME+"/pose",PoseStamped,self.callback)



	def callback(self,msg):
		pwcs = PoseWithCovarianceStamped()
		pwcs.header = msg.header
		pwcs.header.frame_id = "aspen_odom"
		pwcs.pose.pose = msg.pose


		cov = np.diag([0.1,0.1,0.1,0.1,0.1,0.1])
		pwcs.pose.covariance = list(cov.flatten())

		self.pub.publish(pwcs)




if __name__ == "__main__":
	rospy.init_node("vicon_repub")
	vr = ViconRepub()
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()
