#!/usr/bin/python3

"""

This class is used as an interface for the localinos and ROS. It's complimentary arduino program is called localino.ino and can be found in the cohrint's github under the arduino-dw1000 repository. 

Let's first write a simulation with localino.py before implementing the .ino

Future revisions, ask for localino name and do dynamic adding to the ros network with minimal configuration

"""

__author__ = "LT"
__copyright__ = "Copyright 2018, Cohrint"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "LT"
__email__ = "luba6098@colorado.edu"
__status__ = "Development"

import rospy
import serial
import signal
import sys

from std_msgs.msg import Float32

# For 3 robot experiment
_starting_pos3 = {'A' : 1, 'B' : 0, 'D' : 3}
_names = {'A' : 'Case', 'B' : 'Kipp', 'C' : 'Name3', 'D' : 'Turtlebot' }
_talk_to_list3 = {'A' : ['B','D'], 'B' : ['A','D'], 'D' : ['A','B'] }
_transitions3 = ['1','1','7','7']

class Localino():

    def __init__(self):

        self.gk = GracefulKiller()
        ttyStr = '/dev/localino' # udev rule should load this
        self.l = serial.Serial(ttyStr, 9600)
        
        self.letter = 'A'
        rospy.init_node('localino_' + self.letter.upper())
        rospy.loginfo("Starting localino node")

        num_meas = int(rospy.get_param('~num_meas'))
        
        if num_meas > 255:
            rospy.logerr("num_meas param must be <256!")
            sys.exit("num_meas param must be <256")
        
            
        #self.letter = rospy.get_param('~letter').upper()
        ls = rospy.get_param('~localinos').upper()
        localinos = ls.replace(self.letter, "") # remove the current localino's letter if it was included
        self.localinos = dict()

        # Create publisher objects
        for c in localinos: # iterate over each localino letter
            self.localinos[c] = rospy.Publisher(_names[c] + '_dist', Float32, queue_size=10)
        
        num_localinos = len(localinos) + 1 # num of localinos, include ourself in the count
        if num_localinos == 3:
            self.three_robot_experiment(num_meas)
        else:
            rospy.logerr("4 robot experiment not supported yet")
            self.four_robot_experiment()
            
        rospy.loginfo("Killing Localino Node")

    def four_robot_experiment(self):
        pass

    def three_robot_experiment(self, num_meas):
        
        rospy.loginfo("Starting Localino Communication")
        cur_localino = 0
        cur_transition = _starting_pos3[self.letter]
        
        while True:

            # Send the contact letter
            talk_to_letter = _talk_to_list3[self.letter][cur_localino]
            self.l.write(talk_to_letter)
            
            # Send frequency
            self.l.write(_transitions3[cur_transition])

            # Send number of packets
            self.l.write(chr(num_packets))
            self.l.write('|')

            # receive distance from localino
            packet_sum = 0
            error_packets = 0
            for i in range(0,num_meas):
                r = self.parse_packet()
                if r == 0: # check for an error
                    rospy.logerr("Testing Erroneous")
#                    error_packets = error_packets + 1
                if r == 111.11:
                    rospy.logerr("Testing Successful")
                packet_sum = packet_sum + r
            rospy.loginfo("Packet sum: " + str(packet_sum))

            # calculate the average
            range_avg = packet_sum / (num_meas - error_packets)

            # publish the average range
            rospy.logdebug("Publishing " + talk_to_letter + "'s position")
            msg = Float32()
            msg.data = range_avg
            self.localinos[talk_to_letter].publish(msg)

            # udpate to the next index
            cur_localino = ( cur_localino + 1) % len(self.localinos) # talk to next localino
            cur_transition = ( cur_localino + 1) % len(self.localinos) # make frequency transition            
            
            if self.gk.kill_now:
                break
            
        
    def parse_packet(self):
        """ 
        Parses the range packet from localino & returns the floating point value of the range between the localinos
        """
        try: # handle killing during communication or bad packets
            s = self.l.read(6)
            return float(s.decode("utf-8")
        except:
            return 0
        
"""

Write a udev rule

- Load rosparams of the name of the other robots
- Load used letters of the other localinos
- Create ros topics
- 

First the localino should be polling on an 's'
- Send who to contact letter
- Send frequency
- Poll on localino to spit back range data 10 times
- Calculate the avg
- publish the data
- See who's next in the list
- Loop back

We tell the localino what frequency to be on and who it's looking for (who to receive and reject info from)


The localino has its letter


When we kill let's give the localino the 'e' for it loop back up 
"""

# Credits Mayank Jaiswal
class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        
    def exit_gracefully(self,signum, frame):
        self.kill_now = True

if __name__ == "__main__":
    Localino()
