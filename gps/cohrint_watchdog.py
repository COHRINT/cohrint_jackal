#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist 


last_msg = ''
time1 = 0
def callback(data):
	global last_msg
	global time1
	if data.header.stamp.nsecs == last_msg:
		rospy.loginfo("Potentially bad connection for teleop, get closer")
		if (rospy.get_time() - time1)>1:
			rospy.loginfo("No connection for 1 seconds")
			timeout()
	last_msg = data.header.stamp.nsecs
	time1 = rospy.get_time()

def dog():
	rospy.init_node('dog', anonymous=True)
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
