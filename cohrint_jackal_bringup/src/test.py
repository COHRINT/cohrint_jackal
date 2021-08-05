#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

x_vel = None
y_vel = None


def callback(data):
    global x_vel, y_vel
    
    # x = data.pose.pose.position.x * 100
    # y = data.pose.pose.position.y * 100
    # z = data.pose.pose.position.z * 100
    
    # rospy.loginfo("\nx: %f\ny: %f\nz: %f", x,y,z)

    # Extract position from odometry data
    xPos = data.pose.pose.position.x
    yPos = data.pose.pose.position.y
    zPos = data.pose.pose.position.z

    # Extract orientation (quaternion) from odometry data
    orientation_q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

    (yRot, xRot, zRot) = euler_from_quaternion(orientation_q)

    rospy.loginfo("\nPosition: [%f, %f, %f]\nOrientation: [%f,%f,%f]\n\nx_vel: %f, y_vel: %f\n\n",xPos,yPos,zPos,xRot,yRot,zRot,x_vel,y_vel)


def pubnsub():
    global x_vel, y_vel

    rospy.init_node('pubnsub', anonymous=True)
    rate = rospy.Rate(10)

    pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/odometry/filtered', Odometry, callback)

    while not rospy.is_shutdown():
        vel = Twist()
        vel.linear.x = x_vel
        vel.linear.y = y_vel
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        x_vel = float(sys.argv[1])
        y_vel = float(sys.argv[2])
        
        pubnsub()
    except rospy.ROSInterruptException:
        pass