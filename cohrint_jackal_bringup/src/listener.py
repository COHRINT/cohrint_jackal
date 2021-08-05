#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def callback(data):
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

    rospy.loginfo("\nPosition: [%f, %f, %f]\nOrientation: [%f,%f,%f]\n\n",xPos,yPos,zPos,xRot,yRot,zRot)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/odometry/filtered', Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()