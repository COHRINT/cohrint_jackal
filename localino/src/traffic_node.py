#!/usr/bin/python3
"""

This node organizes communication among the localinos.
- dynamically assigns names to the localinos
- coordinates communication among localinos
- Contains procedures to handle one of the localinos dropping out

"""

from std_msgs.msg import String
from std_msgs.msg import UInt8
#from localino.srv import TrafficAddName
from localino.srv import *
from localino.msg import Instruction

import rospy
import rospkg
import roslib

import itertools

class Traffic_Node:

    localinos = {} # dictionary of localinos and corresponding numbers & publishing objects
    localino_combos = []
    localino_combos_index = 0
    # certain number corresponds to this robot
    tag = None
    anchor = None
    
    def __init__(self):

        rospy.init_node("Traffic_Director")
        self.localinos_timeout = rospy.get_param("localinos_timeout", 0.5)
        
        self.server = rospy.Service("add_name_traffic", TrafficAddName, self.add_localino)

        rospy.loginfo("Traffic Director Ready")
        rospy.spin()

    def get_combos(num_localinos):
        l = itertools.combinations(list(range(1,num_localinos + 1)), 2)
        combos = []
        for i in l:
            combos.append(i)
            combos.append(( i[1], i[0] ))
        return combos

    def add_localino(self, req):
        """
        Record the localino's name reply with a number for its localino to use
        """
        robot_name = req.name
        num = len(self.localinos) + 1
        topic = '/' + robot_name + '/instruct'
        pub = rospy.Publisher(topic, Instruction, queue_size=10)
        
        self.localinos[num] = [ robot_name, pub]  # key: num, value=[name, publisher]
        self.localino_combos = self.get_combos(len(self.localinos))
        self.localino_combos_index = 0 # each time restart indexing

        if tag is None and len(self.localinos) is not 1: # start localization
            tag = 1
            anchor = 2
            i = Instruction()
            i.num = anchor
            i.name = self.localinos[anchor][0]
            self.localinos[tag][1].publish(i)
        
        return TrafficAddNameResponse(num, self.localinos_timeout) # return localino number and timeout

    def get_new_tag_anchor():
        """
        Increments by anchor first but flips reverses the measurment 
        E.g. w/ 4 localinos
        [1, 2], [2, 1], [1, 3], [3, 1], [1, 4], [4, 1], [2, 3], [3, 2], [2, 4] ...
        """
        self.localino_combos_index = ( self.localino_combos_index + 1) % len(self.localinos) # increment the index
        return self.localino_combos[self.localino_combos_index]

    def old_increment_tag_anchor(tag, anchor, num_locs):
        """
        Increments the tag anchor by looping through anchors first
        E.g. w/ 4 localinos
        [1, 2], [1, 3], [1, 4], [2, 1], [2, 3], [2, 4], [3, 1] ...
        """
        if tag > num_locs or anchor > num_locs or tag == anchor:
            print("bad input")
            return 1,2
        # first check to increment the anchor
        anchor = (anchor + 1) % ( num_locs + 1)
        if anchor == 0:
            anchor = 1
            tag += 1
        if anchor == tag:
            if not (anchor % num_locs):
                tag = 1
                anchor = 2
            else:
                anchor += 1
        return tag, anchor
        
    def increment_comm(self, msg):
        num = s.data
        if name != self.tag:
            rospy.logwarn("Received an unexpected measurement from " + name)
        self.tag, self.anchor = self.increment_tag_anchor(self.tag, self.anchor, len(self.localinos))

    def timeout(self, msg): # if I can read the timers name or number then I could scale this indefinitely
        """
        Communication failed between two localinos, let's increment and warn
        """
        rospy.logwarn("Communication timeout, tag: " + self.tag + " anchor: " + self.anchor)
        s = String()
        s.data = tag
        self.increment_comm()


if __name__ == "__main__":
    Traffic_Node()
#!/usr/bin/python3

import sys


if __name__ == "__main__":
    tag = int(sys.argv[1])
    anchor = int(sys.argv[2])
    size = int(sys.argv[3])
    print(increment_tag_anchor(tag,anchor,size))
