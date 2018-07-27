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
        rospy.loginfo("Waiting for Traffic Director")
        rospy.wait_for_service('/add_name_traffic')
        try:
            add_localino = rospy.ServiceProxy('/add_name_traffic', TrafficAddName)
            res = add_localino(self.name)
            self.localino_num = res.num
            self.timeout = res.timeout
            rospy.loginfo("Contacted traffic director. My number: " + str(self.localino_num))
        except Exception as e:
            rospy.logerr('Could not connect to Traffic Director Server')
            rospy.logerr(e)
            sys.exit(1)

        # self.localino_num = 1
        # self.timeout = 1.0
        # Localino Serial stuff
        s = rospy.get_param('~dev') # ACM0 or ACM1
        ttyString = '/dev/tty' + s
#        self.l = serial.Serial(ttyStr, 9600, timeout=self.timeout)
        self.l = serial.Serial(ttyString, 9600, timeout=0.1, write_timeout=0.1) # check but don't block
#        self.l = serial.Serial(ttyString, 9600)        
        self.sync_localino()
        self.l.write(chr(self.localino_num).encode()) # send the localino its number
        self.wait_ack()

        rospy.Subscriber('instruct', Instruction, self.instruction) # should take the namespace from launch file
        self.pubComplete = rospy.Publisher('/meas_complete', String, queue_size=10)
        self.pubDist = rospy.Publisher('distances', Distance, queue_size=10)
        rospy.loginfo(self.name + "'s localino node ready")

    def wait_ack(self):
        """ We could block in here... """
        rospy.loginfo("Waiting for ack from localino...")
        resets = 0
        delayTime = 0.5
        rospy.sleep(delayTime / 2)
        while not rospy.is_shutdown():
            try:
                c = self.l.read(1).decode('utf-8')
                if c == 'a':
                    c2 = self.l.read(1).decode('utf-8')
                    rospy.loginfo("Received ack #" + str(c2))
                    break
                elif not c:
                    rospy.logwarn("No ack from localino")
                    resets += 1
                    rospy.sleep(delayTime)
                else:
                    rospy.logwarn("Expecting ack code from localino, received other char: " + c)
                if ( resets * delayTime ) > self.timeout :
                    rospy.warn("Timed out waiting for ack, resetting")
                    return 1
            except Exception as e:
                rospy.logwarn(e)
            rospy.sleep(delayTime)
            return 0
#        rospy.loginfo("Received ack")

    def parse_packet(self):
        prev = self.l.timeout # record previous state of self.l.timeout
        self.l.timeout = None # let's block on receiving packets for now
        """ 
        Parses the range packet from localino & returns the floating point value of the range between the localinos
        """
        try: # handle killing during communication or bad packets
            s = self.l.read(6)
            rospy.loginfo("Packet Contents: " + s.decode("utf-8"))
            self.l.timeout = prev
            return float(s.decode("utf-8"))
        except Exception as e:
            rospy.logwarn(e)
            self.l.timeout = prev            
            return -1.0
                         
    def instruction(self, msg):
        rospy.loginfo("Received instruction from traffic director")
        num = int(msg.num)
        name = msg.name
        freq = int(msg.freq)

        self.reset_localino()
        rospy.sleep(0.2)

        rospy.loginfo("Sending instruction to the localino")
        try:
            self.l.write(b'i') # send localino instruction char 
            self.l.write(chr(num).encode()) # send number of who to contact
        except SerialTimeoutException as e:
            rospy.logwarn(e)
            self.reset_localino() # localino should be in anchor state
            self.l.write(b'a') # tell localino to switch back to anchor
            self.wait_ack()
            return
        
        if self.wait_ack():
            return

        # start the local timer
        self.timedOut = False
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.timeout),self.local_timeout)

        if not self.interpret_localino():
            r = self.parse_packet() # get the localino distance
            if r > 0:
                rospy.loginfo("range: " + str(r))
                d = Distance()
                d.toWhom = name
                d.dist = r
                self.pubDist.publish(d)
                
                # Inform Traffic Director we completed
                s = String()
                s.data = self.name
                self.pubComplete.publish(s)
            else:
                rospy.logerr("Error in localino range packet")
        self.l.write(b'a') # tell localino to switch back to anchor
        self.wait_ack()
        self.reset_localino() # localino should be in anchor state

    def interpret_localino(self):
        while not rospy.is_shutdown():
            c = self.l.read(1).decode('utf-8')
            if c == 'p':
                rospy.loginfo("poll sent")
            elif c == 'o':
                rospy.loginfo("poll ack received")
            elif c == 'r':
                rospy.loginfo("range sent")
            elif c == 'c':
                rospy.loginfo("got the range report!")
                self.timer.shutdown()
                return 0
            elif c == 't':
                rospy.loginfo("on board localino timeout")
            elif c == 'u':
                rospy.loginfo("unknown char..")
            elif c == 'R':
                rospy.loginfo("Tag received something...")
            elif c == 'n':
                rospy.loginfo("Msg wasn't for me...")
            elif c == 'N':
                rospy.logwarn("Msg wasn't from who I thought...")
                other_num = self.l.read(1).decode()
                sender = self.l.read(1).decode()
                rospy.loginfo("O.N. : " + other_num + " sender : " + sender)
            elif c == 'G':
                rospy.loginfo("Msg for Me! And from whom I thought.")
            elif c == 'l':
                rospy.loginfo("Localino looping")
            elif c in ['0','1','2','3','4','5','6','7','8','9']:
                rospy.loginfo("Received a # char..." + c)
            elif c == "M":
                c2 = self.l.read(1).decode()
                rospy.loginfo("Localino thinks its # is " + c2)
                c3 = self.l.read(1).decode()
                rospy.loginfo("Other # is " + c3)
            elif c == 'e': # Error localino knows about
                self.timer.shutdown()                
                c2 = int(self.l.read(1).decode('utf-8'))
                rospy.logwarn("Localino Error: " + str(c2))
                return -1
            elif self.timedOut: # let's tell the localino to change back to anchor
#                rospy.logwarn("Received timer timeout, switching to anchor.")
                return -2
            elif c == '':
                pass
            else:
                rospy.logerr("Received unknown response of type: " + str(type(c)))
                rospy.logerr("info received: " + str(c))
                
    def local_timeout(self, msg):
        """
        If we don't get a response from the localino, let's 
        """
        self.timer.shutdown()
        rospy.logwarn("Node Timeout waiting for localino")
        self.timedOut = True
            
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
                if c == '':
                    continue
                ck = c.decode('utf-8') # decode the char
                if ck == syncCode:
                    break
                else:
                    rospy.loginfo("Received other char from localino: " + ck)
            except Exception as err:
                rospy.logwarn(err)
            rospy.sleep(1)
        rospy.loginfo("Localino Synced")
                
if __name__ == "__main__":
    rospy.init_node('localino_node', anonymous=True)
    l = Localino()
    rospy.spin()
