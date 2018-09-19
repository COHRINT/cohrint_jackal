#!/usr/bin/env python
"""

This node organizes communication among the localinos.
- at runtime assigns names to the localinos that have done a rosservice request
- coordinates communication among localinos by publishing to the /localino_name/instruct topic where localino_name is the name given in the rosservice request
- Contains procedures to handle one of the localinos dropping out

"""

from std_msgs.msg import String
from std_msgs.msg import UInt8
from localino.msg import *
from localino.srv import *
import rospy
import itertools

# for use with self.localinos dictionary of 2 item lists (first item being the name, 2nd item the publisher object)
NAME_INDEX = 0
PUBLISHER_INDEX = 1

HARDCODE_CHANNEL = 1 # The communication scheme was designed with the flexibility to allow for communication on multiple channels, but for our purposes we get fast enough communication just on one channel

class Traffic_Node:

    localinos = {} # dictionary of localinos and corresponding numbers & publishing objects
    localino_combos = []
    localino_combos_index = 0
    # certain number corresponds to this robot
    tag = None
    anchor = None
    
    def __init__(self):
        
        self.traffic_timeout = float(rospy.get_param("~traffic_timeout", 1.0))
        self.localinos_timeout = float(rospy.get_param("~localinos_timeout", 0.5))

        # initialize ros objects
        self.server = rospy.Service("/add_name_traffic", TrafficAddName, self.add_localino)
        rospy.Subscriber('/meas_complete', String, self.increment_comm)
        rospy.loginfo("Traffic Director Ready")
        

    def get_combos(self, num_localinos):
        """
        Returns all of the communication combinations possible, given the number of added localinos
        """
        l = itertools.combinations(list(range(1,num_localinos + 1)), 2)
        combos = []
        for i in l:
            combos.append(i)
            combos.append(( i[1], i[0] ))
        print(list(combos))
        return combos

    def check_existing(self, name):
        """ If a localino already exists, returns its number """
        for i in range(1, len(self.localinos) + 1):
            if name == self.localinos[i][NAME_INDEX]:
                rospy.logwarn(name + " has already been added, skipping")
                return i
        return False

    def add_localino(self, req):
        """
        Record the localino's name reply with a number for its localino to use
        """
        robot_name = req.name

        # check if the localino has already been added
        existing_num = self.check_existing(robot_name)
        if existing_num:
            return existing_num, self.localinos_timeout
        
        num = len(self.localinos) + 1
        topic = '/' + robot_name + '/instruct'
        pub = rospy.Publisher(topic, Instruction, queue_size=10)
        
        self.localinos[num] = [ robot_name, pub ]  # key: num, value=[name, publisher]
        self.localino_combos = self.get_combos(len(self.localinos))
        self.localino_combos_index = -1 # each time restart indexing
        rospy.loginfo("Adding " + robot_name + " to known localinos.\tNum Localinos : " + str(len(self.localinos)))
        
        # start localization
        if self.tag is None and len(self.localinos) is not 1:
            self.tag = 1
            self.anchor = 2
            self.pub_tag_anchor()

        return num, self.localinos_timeout # return localino number and timeout

    def get_new_tag_anchor(self):
        """
        Increments by anchor first and reverses for the following measurment 
        E.g. w/ 4 localinos
        [1, 2], [2, 1], [1, 3], [3, 1], [1, 4], [4, 1], [2, 3], [3, 2], [2, 4] ...
        """
        self.localino_combos_index = ( self.localino_combos_index + 1) % len(self.localino_combos) # increment the index
        return self.localino_combos[self.localino_combos_index]
        
    def increment_comm(self, msg):
        """
        Increment the communication to the next two localinos
        """
        self.timer.shutdown()
        name = msg.data
        if name != self.localinos[self.tag][NAME_INDEX]:
            rospy.logwarn("Received an unexpected measurement from " + str(name))
        self.tag, self.anchor = self.get_new_tag_anchor()
        self.pub_tag_anchor()

    def pub_tag_anchor(self):
        """ 
        Publishes the tag (self.tag) and anchor (self.anchor) to start communication between the two localinos 
        - Starts the comm timeout timer also
        """
        i = Instruction()
        i.num = self.anchor
        i.name = self.localinos[self.anchor][NAME_INDEX]
        i.freq = HARDCODE_CHANNEL # we'll just use one frequency for our purposes
        self.localinos[self.tag][PUBLISHER_INDEX].publish(i)
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.traffic_timeout),self.timeout) # we'll expect a reply from the localino node within a certain period of time or we can assume that we've lost connection and can increment
        
        tag = self.localinos[self.tag][NAME_INDEX]
        tagNum = str(self.tag)
        anchor = i.name
        anchorNum = str(self.anchor)
        rospy.loginfo("Instructing " + tag + "(" + tagNum + ") to contact " + anchor + "(" + anchorNum + ")")

    def timeout(self, msg):
        """
        Communication failed between two localinos, let's increment and warn
        - TODO create a 2nd timer for parallel communication (one new timer for each new channel)
        """
        # print to screen localinos that failed to contact
        tag = self.localinos[self.tag][NAME_INDEX]
        anchor = self.localinos[self.anchor][NAME_INDEX]
        rospy.logwarn("Communication timeout, \ttag : " + tag + "\tanchor : " + anchor)

        # increment to the next comm pair
        s = String()
        s.data = self.localinos[self.tag][NAME_INDEX]
        self.increment_comm(s)


if __name__ == "__main__":
    rospy.init_node("Traffic_Director")
    t = Traffic_Node()
    rospy.spin()
