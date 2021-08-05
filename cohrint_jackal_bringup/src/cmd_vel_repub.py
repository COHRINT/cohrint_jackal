#!/usr/bin/env python
import tf2_ros
import tf2_geometry_msgs
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
import numpy as np



class VelRepub:
	def __init__(self):

		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		rospy.Subscriber("/jackal_velocity_controller/odom",Odometry,self.odom_callback)
		self.twist_pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel/cov",TwistWithCovarianceStamped,queue_size=10)


	def odom_callback(self,msg):
		twist = TwistWithCovarianceStamped()
		twist.twist = msg.twist
		twist.header = msg.header
		twist.header.frame_id = "base_link"

		self.twist_pub.publish(twist)


	def odom_callback_old(self,msg):
		# Convert linear velocity to a vector 3 message
		odom_v3 = Vector3Stamped()
		odom_v3.header = msg.header 
		odom_v3.vector = msg.twist.twist.linear
		
		print(str(odom_v3.vector.x)+ " , " + str(odom_v3.vector.y)+ " , " +str(odom_v3.vector.z))
		target_frame = "base_link"
		transform = self.tf_buffer.lookup_transform(target_frame,
                                       odom_v3.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

		base_link_v3 = tf2_geometry_msgs.do_transform_vector3(odom_v3,transform)

		base_link_twist = TwistWithCovarianceStamped()
		base_link_twist.header = base_link_v3.header
		base_link_twist.twist.twist.linear = base_link_v3.vector

		cov = np.diag([0.1,0.1,0.1,0.1,0.1,0.1])
		base_link_twist.twist.covariance = list(cov.flatten())
		self.twist_pub.publish(base_link_twist)



if __name__ == "__main__":
	rospy.init_node("vel_repub")
	vr = VelRepub()
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()


