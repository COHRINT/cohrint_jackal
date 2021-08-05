#!/usr/bin/env python
# Import necessary libraries
import rospy # Ros library
import math # Math library
import sys
from geometry_msgs.msg import Twist # Twist messages
from nav_msgs.msg import Odometry # oOdometry messages
from tf.transformations import euler_from_quaternion # Quaternion conversions

# Initialize global variables
x_dest = None # Stores the x-coordinate of the destination
y_dest = None # Stores the y-coordinate of the destination
dist_err = 1 # Stores the distance error
pose_cmd_vel = None # command velocity
rot_cmd_vel = None # angular command velocity
K1 = 2 # Angle gain
K2 = 4 # Distance gain

def calculate_heading(data):
    # Initialize global variables
    global x_dest, y_dest, K1, K2, pose_cmd_vel, rot_cmd_vel, dist_err
    
    # Extract position from odometry data
    x_pose = data.pose.pose.position.x # Current x-position of the Jackal
    y_pose = data.pose.pose.position.y # Current y-position of the Jackal
    z_pose = data.pose.pose.position.z # Current z-position of the Jackal
 
    # Extract orientation (quaternion) from odometry data
    orientation_q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    
    # Convert quaternion to euler angles
    (y_rot, x_rot, z_rot) = euler_from_quaternion(orientation_q)

    # Calculate x and y errors
    x_err = x_dest - x_pose
    y_err = y_dest - y_pose

    # Calculate angular error
    angle_err = math.atan2(y_err,x_err)

    if angle_err > math.pi:
        angle_err = angle_err - (2 * math.pi)
 
    angle_err = angle_err - z_rot

    if angle_err > math.pi:
        angle_err = angle_err - (2 * math.pi)
    elif angle_err < -math.pi:
        angle_err = angle_err + (2 * math.pi)
 
    # Calculate distance error
    dist_err = math.sqrt((x_err ** 2) + (y_err ** 2))
 
    # Calculate command velocities
    if abs(angle_err) > 0.05: # Only rotate
        rot_cmd_vel = angle_err * K1
        pose_cmd_vel = 0
    else: # Rotate and move
        rot_cmd_vel = angle_err * K1
        pose_cmd_vel = dist_err * K2
 
    if pose_cmd_vel > 1.0:
        pose_cmd_vel = 1.0
 
def go_to_waypoint():
    # Initialize global variables
    global x_dest, y_dest, K1, K2, pose_cmd_vel, rot_cmd_vel, dist_err
    
    # Initialize ros node
    rospy.init_node('go_to_waypoint', anonymous=True)
    # Set sleep rate
    sleep_rate = rospy.Rate(10)
  
    # Initialize odometry subscriber
    grab_odom = rospy.Subscriber('/odometry/filtered/aspen', Odometry, calculate_heading)
    # Initialize command velocity publisher
    pub_vel = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
  
    # Go to waypoint
    while dist_err > 0.1:
        vel = Twist()
        vel.linear.x = pose_cmd_vel
        vel.angular.z = rot_cmd_vel
        pub_vel.publish(vel)
        sleep_rate.sleep()
 
if __name__ == '__main__':
    try:
        x_dest = float(sys.argv[1])
        y_dest = float(sys.argv[2])
 
        go_to_waypoint()
    except rospy.ROSInterruptException:
        pass
