#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class SimpleBox:
	def __init__(self):
		self.pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel",Twist,queue_size=10)
		self.BOX_SIZE = 50
		self.ANGLE_SIZE = 40
		self.speed = 0.3
		self.ang_speed = 0.4

		self.forward(self.BOX_SIZE)
		self.turn(self.ANGLE_SIZE)
		self.forward(self.BOX_SIZE)
		self.turn(self.ANGLE_SIZE)
		self.forward(self.BOX_SIZE)
		self.turn(self.ANGLE_SIZE)
		self.forward(self.BOX_SIZE)
		self.turn(self.ANGLE_SIZE)


	def forward(self,dist):
		rate = rospy.Rate(10)
		for i in range(dist):
			t = Twist()
			t.linear.x = self.speed
			self.pub.publish(t)
			rate.sleep()

	def turn(self,ang):
		rate = rospy.Rate(10)
		for i in range(ang):
			t = Twist()
			t.angular.z = self.ang_speed
			self.pub.publish(t)
			rate.sleep()


if __name__ == "__main__":
	rospy.init_node("simple_box")
	sb = SimpleBox()

