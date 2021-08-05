#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
import numpy as np 


class CmdRepub:
	def __init__(self):
		self.pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel_cov",TwistWithCovarianceStamped,queue_size=10)

		rospy.Subscriber("/jackal_velocity_controller/cmd_vel",Twist,self.callback)

		self.twist = Twist()

		self.run()


	def callback(self,msg):
		self.twist = msg


	def make_twist_cov(self):

		twcs = TwistWithCovarianceStamped()
		twcs.header.frame_id = "base_link"
		twcs.header.stamp = rospy.Time.now()

		twcs.twist.twist = self.twist


		cov = np.diag([0.1,-1,-1,-1,-1,0.1])
		twcs.twist.covariance = list(cov.flatten())

		return twcs


	def run(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub.publish(self.make_twist_cov())
			self.twist = Twist()
			rate.sleep()



if __name__ == "__main__":
	rospy.init_node("cmd_repub")
	cr = CmdRepub()
