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
from std_msgs.msg import UInt8, String

from pdb import set_trace

ttyStr = '/dev/localino' # udev rule (67-localino-rules) loads this
syncCode = 's'

class Localino:

    localino_num = 0
    timeout = 0
    
    def __init__(self):

        self.name = rospy.get_param('~robot_name')
        rospy.loginfo("Starting " + self.name + "'s Localino Node")
        # rospy.loginfo("Waiting for Traffic Director")
        # rospy.wait_for_service('/add_name_traffic')
        # try:
        #     add_localino = rospy.ServiceProxy('/add_name_traffic', TrafficAddName)
        #     res = add_localino(self.name)
        #     self.localino_num = res.num
        #     self.timeout = res.timeout
        #     rospy.loginfo("Contacted traffic director")
        # except Exception as e:
        #     rospy.logerr('Could not connect to Traffic Director Server')
        #     rospy.logerr(e)
        #     sys.exit(1)

        

        self.localino_num = 1
        self.timeout = 1.0
        # Localino Serial stuff
        s = rospy.get_param('~dev') # ACM0 or ACM1
        ttyString = '/dev/tty' + s
#        self.l = serial.Serial(ttyStr, 9600, timeout=self.timeout)
        # self.l = serial.Serial(ttyString, 9600, timeout=self.timeout)
        self.l = serial.Serial(ttyString, 9600)        
        self.sync_localino()
        self.l.write(chr(self.localino_num).encode()) # send the localino its number
        self.wait_ack()

        rospy.Subscriber('instruct', Instruction, self.instruction) # should take the namespace from launch file
        self.pubComplete = rospy.Publisher('/meas_complete', String, queue_size=10)
        self.pubDist = rospy.Publisher('distances', Distance, queue_size=10)
        rospy.loginfo(self.name + "'s localino node ready")

    def wait_ack(self):
        """ We could block in here... """
        while not rospy.is_shutdown():
            try:
                c = self.l.read(1)
                if c.decode('utf-8') == 'a':
                    break
                elif not c:
                    rospy.logwarn("Timed out awaiting ack from localino")
                else:
                    rospy.logwarn("Expecting ack from localino, received other char")
            except Exception as e:
                rospy.logwarn(e)
        rospy.loginfo("Received ack")

    def parse_packet(self):
        self.l.timeout = None # let's block on receiving packets for now
        """ 
        Parses the range packet from localino & returns the floating point value of the range between the localinos
        """
        try: # handle killing during communication or bad packets
            s = self.l.read(7)
            rospy.loginfo("Packet Contents: " + s.decode("utf-8"))
            return float(s.decode("utf-8"))
        except Exception as e:
            rospy.logwarn(e)
            return -1.0
                         
    def instruction(self, msg):
        rospy.loginfo("Received instruction from traffic director")
        num = msg.num
        name = msg.name
        freq = msg.freq

        self.reset_localino()
        rospy.sleep(0.2)

        self.l.write(b'i')
        self.l.write(chr(num).encode()) # tell localino who to contact
        self.l.write(chr(freq).encode()) # tell localino which frequency
        self.wait_ack()

        r = self.parse_packet() # get the localino distance
        print("range: " + str(r))
#        set_trace()
        if r == 0:
            rospy.logwarn("Localino serial timeout")
        elif r == -1:
            rospy.logwarn("Error in communication w/ localino")
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
        rospy.loginfo("Attempting to sync with localino")
        while not rospy.is_shutdown():
            self.reset_localino()
            try: # handle the sigterm exception 
                self.l.write(syncCode.encode()) # send start condition
                c = self.l.read(1) 
                ck = c.decode('utf-8') # decode the char
                if ck == syncCode:
                    break
                else:
                    rospy.logdebug("Received other char from localino")
            except Exception as err:
                rospy.logwarn(err)
        rospy.loginfo("Localino Synced")
                
if __name__ == "__main__":
    rospy.init_node('localino_node', anonymous=True)
    l = Localino()
    rospy.spin()
"""

When we contact a localino, we want to include: address and return address

"""
