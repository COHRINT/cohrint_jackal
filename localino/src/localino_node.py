#!/usr/bin/env python
"""

This node serves as the interface between the traffic node and the localinos
- Asks traffic node for a number
- listens to traffic node's instructions for who to communicate with
- handles serial communication to localino
- publishes to /localino_nameA_to_nameB where nameA is the tag who traffic node said to initiate communication, nameB is who the traffic node told nameA to communicate with

"""

import rospy
import sys
import serial
from localino.msg import *
from localino.srv import *
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Float32


ttyStr = '/dev/localino' # udev rule (67-localino-rules) loads this
syncCode = 's'

class Localino:

    localino_num = 0
    timeout = 0
    
    def __init__(self):
        rospy.init_node('localino_node', anonymous=True)
        self.name = rospy.get_param('~robot_name')
        rospy.loginfo("Starting " + self.name + "'s Localino Node")
        rospy.loginfo("Waiting for Traffic Director")
        rospy.wait_for_service('/add_name_traffic')
        try:
            add_localino = rospy.ServiceProxy('/add_name_traffic', TrafficAddName)
            res = add_localino(self.name)
            self.localino_num = res.num
            self.timeout = res.timeout
            rospy.loginfo("Contacted traffic director")
        except Exception as e:
            rospy.logerr('Could not connect to Traffic Director Server')
            rospy.logerr(e)
            sys.exit(1)

        # begin communication w/ Localino
        self.l = serial.Serial(ttyStr, 9600, timeout=self.timeout)
        self.sync_localino()
        self.l.write(chr(self.localino_num).encode()) # send the localino its number
        
        rospy.Subscriber('instruct', Instruction, self.instruction) # should take the namespace from launch file
        self.pubComplete = rospy.Publisher('/meas_complete', String, queue_size=10)
        self.pubDist = rospy.Publisher('distances', Distance, queue_size=10)
        
        rospy.spin()

    def parse_packet(self):
        """ 
        Parses the range packet from localino & returns the floating point value of the range between the localinos
        """
        try: # handle killing during communication or bad packets
            s = self.l.read(7)
            return float(s.decode("utf-8"))
        except:
            return 0        
                         
    def instruction(self, msg):
        num = msg.num
        name = msg.name
        freq = msg.freq

        self.reset_localino()
        self.l.write(chr(num).encode())
        self.l.write(ch

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
            
        self.reset_localino() # localino should be in anchor state
            
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


if __name__ == "__main__":
    Localino()

"""

When we contact a localino, we want to include: address and return address

"""
