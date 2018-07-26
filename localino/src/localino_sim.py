#!/usr/bin/env python

""" File simply for use in debugging a localino's first instruction """

import rospy
import sys
from localino.msg import *
from localino.srv import *
from std_msgs.msg import UInt8, String

class LocalinoSim():
    def __init__(self, name):
        self.name = name
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

        topic = '/' + name + '/instruct'
        rospy.Subscriber(topic, Instruction, self.instruction) # should take the namespace from launch file
        self.pubComplete = rospy.Publisher('/meas_complete', String, queue_size=10)
        self.pubDist = rospy.Publisher('distances', Distance, queue_size=10)
        rospy.loginfo(self.name + "'s localino node ready")
        rospy.spin()

    def instruction(self, msg):
        rospy.loginfo("Received instruction from traffic director")
        num = int(msg.num)
        name = msg.name
        freq = int(msg.freq)

        rospy.sleep(0.5)
        rospy.loginfo("Sending instruction to the localino")

        r = 0.989
        rospy.loginfo("range: " + str(r))
        d = Distance()
        d.toWhom = name
        d.dist = r
        self.pubDist.publish(d)
                
        # Inform Traffic Director we completed
        s = String()
        s.data = self.name
        self.pubComplete.publish(s)

if __name__ == "__main__":    
    rospy.init_node('localino_node', anonymous=True)
    s = LocalinoSim(sys.argv[1])
    rospy.spin()
