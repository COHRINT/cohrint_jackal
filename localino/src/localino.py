#!/usr/bin/python3
"""

This node serves as the interface between the traffic node and the localinos
- Asks traffic node for a number
- listens to traffic node's instructions for who to communicate with
- handles serial communication to localino
- publishes to /localino_nameA_to_nameB where nameA is the tag who traffic node said to initiate communication, nameB is who the traffic node told nameA to communicate with

"""

import rospy
import sys

#from localino.srv import TrafficAddName
from localino.srv import *
from std_msg.msg import UInt8
from std_msg.msg import String
from std_msg.msg import Float32
from localino.msg import Instruction
from localino.msg import Distance

ttyStr = '/dev/localino' # udev rule (67-localino-rules) loads this
syncCode = 's'

class Localino:

    def __init__(self):
        self.name = rospy.get_param('robot_name')
        rospy.init_node(self.name + '_localino_node')
        rospy.wait_for_service('add_name_traffic')
        try:
            add_localino = rospy.ServiceProxy('add_name_traffic', TrafficAddName)
            self.localino_num, self.timeout = add_localino(self.name)
        except:
            rospy.logerr('Could not connect to Traffic Director Server')
            sys.exit(1)

        rospy.loginfo(self.localino_num)
        rospy.loginfo(self.timeout)
        sys.exit(0)

        self.l = serial.Serial(ttyStr, 9600, timeout=self.timeout)
        self.sync_localino()
        
        self.l.write(self.localino_num) # send the localino its number

        rospy.Subscriber('instruct', Instruction, self.instruction) # should take the namespace from launch file
        self.pubComplete = rospy.Publisher('/meas_complete', String, queue_size=10)
        self.pubDist = rospy.Publisher('distances', Distance, queue_size=10)
        
        rospy.spin()

    def parse_packet(self):
        """ 
        Parses the range packet from localino & returns the floating point value of the range between the localinos
        """
        try: # handle killing during communication or bad packets
            s = self.l.read(6)
            return float(s.decode("utf-8"))
        except:
            return 0        
                         
    def instruction(self, msg):
        num = msg.num
        name = msg.name
        
        self.l.write(chr(num))

        r = self.parse_packet()
        if r == 0:
            rospy.logwarn("No packet from Localino")
        else: # received successful packet from localino
            rospy.logdebug("Received packet from localino")
            d = Distance()
            d.toWhom = name
            d.dist = r
            self.pubDist.publish(d)

            # Inform Traffic Director we completed
            ui = UInt8()
            ui.data = self.localino_num
            self.pubComplete(ui)
            
        reset_localino() # localino should be in anchor state
            
    def reset_localino(self):
        self.l.reset_input_buffer()
        self.l.reset_output_buffer()
        
    def sync_localino(self):
        """ Syncs w/ the localino """
        while True:
            self.reset_localino()
            try: # handle the sigterm exception 
                self.l.write(syncCode.encode()) # send start condition
                c = self.l.read(1) # block on receiving a char
                ck = c.decode('utf-8') # decode the char
                if ck == syncCode:
                    break
                else:
                    rospy.logdebug("Received other char from localino")
            except Exception as err:
                rospy.logwarn(err)
                
        

    def measure(self, msg):
        toContact = msg.data # number of localino to contact
        # send number to localino




"""

When we contact a localino, we want to include: address and return address

"""
