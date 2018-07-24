#!/usr/bin/env python

""" File simply for use in debugging a localino's first instruction """

import rospy
import sys
from localino.msg import *
from localino.srv import *
from std_msgs.msg import UInt8, String

if __name__ == "__main__":
    rospy.init_node('localino_node', anonymous=True)

    name = 'lagrange'
    rospy.loginfo("Starting " + name + "'s Localino Node")
    rospy.loginfo("Waiting for Traffic Director")
    rospy.wait_for_service('/add_name_traffic')
    try:
        add_localino = rospy.ServiceProxy('/add_name_traffic', TrafficAddName)
        res = add_localino(name)
        localino_num = res.num
        timeout = res.timeout
        rospy.loginfo("Contacted traffic director. My number: " + str(localino_num))
    except Exception as e:
        rospy.logerr('Could not connect to Traffic Director Server')
        rospy.logerr(e)
        sys.exit(1)
    rospy.loginfo("Successful, looping")
    while not rospy.is_shutdown():
        pass
