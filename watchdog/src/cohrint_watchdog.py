#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist 


last_msg = ''

def callback(data):
	global last_msg

	if data.header.stamp.nsecs == last_msg:
		rospy.loginfo("Potentially bad connection for teleop, get closer")
		#rospy.loginfo(rospy.get_time())
		#rospy.loginfo(data.header.stamp.secs + data.header.stamp.nsecs)
		if (rospy.get_time() - data.header.stamp.secs) >1.5:
			rospy.loginfo("No connection for at least 1.5 seconds")
			timeout()
	last_msg = data.header.stamp.nsecs
	time1 = rospy.get_time()

def dog():
	rospy.init_node('dog', anonymous=False)
	rospy.Subscriber("/case/bluetooth_teleop/joy", Joy, callback)
	rospy.spin()

def timeout():
	pub = rospy.Publisher("/case/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)
	geo = Twist()
	geo.linear.x = 0.0
	geo.linear.y = 0.0
	geo.linear.z = 0.0
	geo.angular.x = .0
	geo.angular.y = 0.0
	geo.angular.z = 0.0
	pub.publish(geo)
	

if __name__== '__main__':
	dog()
