#!/usr/bin/env python

"""

Records localino data at certain intervals and stores 

To Record:
launch this node
launch the localinos
launch the traffic director
publish to /labelDistance an unsigned integer of the current distance between the localinos

republish to /labelDistance to record more measurements

"""

import rospy
import csv
from localino.msg import Distance
from std_msgs.msg import UInt8
# graceful killer here

class Recorder():

    def __init__(self, localinoName1, localinoName2, numMeas):
        """
        @param:
        localinoName1: string, name of the first localino to record
        localinoName2: string, name of the second localino to record
        numMeas: int, number of measurements to record per distance
        """
        self.localino1 = localinoName1
        self.localino2 = localinoName2
        self.numMeas = numMeas

        topic1 = '/' + self.localino1 + '/distances'
        topic2 = '/' + self.localino2 + '/distances'
        rospy.Subscriber(topic1, Distance, self.distCallback1)
        rospy.Subscriber(topic2, Distance, self.distCallback2)

        recordDistTopic = '/label_distance'
        rospy.Subscriber(recordDistTopic, UInt8, self.labelDistCallback)

        self.meas1 = []
        self.numMeas1 = 0 # number of current measurements on the present distance

        self.meas2 = []
        self.numMeas2 = 0 # number of current measurements on the present distance

        self.curDist = 0
        self.distances = []
        self.recording1 = False
        self.recording2 = False        

    def labelDistCallback(self, msg):
        if self.recording1 or self.recording2:
            rospy.logerr("Haven't finished recording measurements for the present distance and you've asked for another measurement! Wait a sec")
            return
        if msg.data > 100 or msg.data < 0: # check for bad distance
            rospy.logerr("Malformed input on distance between the localinos: " + str(msg.dist))
        self.curDist = msg.data
        self.recording1 = True
        self.recording2 = True
        self.numMeas1 = 0
        self.numMeas2 = 0

    def distCallback1(self, msg):
        if msg.toWhom != self.localino2: # check if this was a measurement of localino 2
            rospy.logerr("recorded message for another localino. Are you running more than 2 localinos?")
        elif not self.recording1 : # ignore the msg
            pass
        else:
            rospy.loginfo("Recording " + self.localino1)
            self.meas1.append(msg.dist)
            self.distances.append(self.curDist)
            self.numMeas1 += 1
            if self.numMeas1 > self.numMeas :
                self.recording1 = False

    def distCallback2(self, msg):
        if msg.toWhom != self.localino1: # check if this was a measurement of localino 1
            rospy.logerr("recorded message for another localino. Are you running more than 2 localinos?")
        elif not self.recording2 : # ignore the msg
            pass
        else:
            rospy.loginfo("Recording " + self.localino2)
            self.meas2.append(msg.dist)
            self.numMeas2 += 1
            if self.numMeas2 > self.numMeas :
                self.recording2 = False

if __name__ == "__main__":
    rospy.init_node('localino_recorder')

    # load params
    localinoName1 = rospy.get_param('~loc1')
    localinoName2 = rospy.get_param('~loc2')
    numMeas = int(rospy.get_param('~numMeas'))
    
    r = Recorder(localinoName1, localinoName2, numMeas)
    rospy.spin()
    print("reached here")
    print(str(r.distances))
    print(str(r.meas1))
    print(str(r.meas2))
    newList = list(zip(r.distances, r.meas1, r.meas2))
    print(newList)
    fileName = localinoName1 + '_' + localinoName2 + '.csv'
    print(fileName)
    with open(fileName, "w+") as data:
        csvWriter = csv.writer(data)
        csvWriter.writerows(newList)
        print("SAVING")

    # can we assume that it's been killed and we're no longer spinning??? or we store it on every final callback)
