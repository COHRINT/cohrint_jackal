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
K1 = 4.4 # Angle gain
K2 = 4 # Distance gain
 
def calculate_heading(data):
   # Initialize global variables
   global x_dest, y_dest, K1, K2, pose_cmd_vel, rot_cmd_vel, dist_err
  
   # Extract position from odometry data
   x_pose = data.pose.pose.position.x # Current x-position of the Jackal
   y_pose = data.pose.pose.position.y # Current y-position of the Jackal
   z_pose = data.pose.pose.position.z # Current z-position of the Jackal

   # print(x_pose)
   # print(y_pose)
   # print("\n")
 
   # Extract orientation (quaternion) from odometry data
   orientation_q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
 
   # Convert quaternion to euler angles
   (y_rot, x_rot, z_rot) = euler_from_quaternion(orientation_q)

   # print(z_rot)
 
   # Calculate x and y errors
   x_err = x_dest - x_pose
   y_err = y_dest - y_pose

   # print(x_err)
   # print(y_err)

   # print(math.atan(y_err / x_err))
 
   # Calculate angular error
   if x_err > 0:
       angle_err = math.atan(y_err / x_err)
   elif x_err < 0:
       if y_err > 0:
           angle_err = math.pi + math.atan(y_err / x_err)
       else:
           angle_err = (-1 * math.pi) + math.atan(y_err / x_err)
   else:
       if y_err > 0:
           angle_err = math.pi / 2
       else:
           angle_err = math.pi / -2.0
 
   # print(angle_err)

   angle_err = angle_err - z_rot

   # print(angle_err)
   # print("\n")

   # Calculate distance error
   dist_err = math.sqrt((x_err ** 2) + (y_err ** 2))

   # print(dist_err)
   # print("\n")
  
   # Calculate command velocities
   if abs(angle_err) > 0.05: # Rotate
       rot_cmd_vel = angle_err * K1
       pose_cmd_vel = 0
   else: # Move
       rot_cmd_vel = angle_err * K1
       pose_cmd_vel = dist_err * K2

   # print(rot_cmd_vel)
   # print("\n")
 
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
   grab_odom = rospy.Subscriber('/odometry/filtered', Odometry, calculate_heading)
   # Initialize command velocity publisher
   pub_vel = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)

   # Go to waypoint
   while dist_err > 0.025:
       print(dist_err)
       print("\n")

       vel = Twist()
       vel.linear.x = pose_cmd_vel
       vel.angular.z = rot_cmd_vel
       # print(vel.angular.z)
       # print("\n")
       pub_vel.publish(vel)
       sleep_rate.sleep()

 
if __name__ == '__main__':
   try:
       x_dest = float(sys.argv[1])
       y_dest = float(sys.argv[2])

       go_to_waypoint()
   except rospy.ROSInterruptException:
       pass