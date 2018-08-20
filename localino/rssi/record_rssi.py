#!/usr/bin/env python

"""

Records the position of the localino in xyz space with the receiving intensity from the localino. There are 3 cases to analyze: 1) Both localinos are parallel in the horizontally
2) Both localinos are parrallel in the vertically
3) One localino is vertical and one localino is horizontal

"""

import signal
import serial
import rospy
import csv

testType = 'vert_vert'

# Credits Mayank Jaiswal
class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        
    def exit_gracefully(self,signum, frame):
        self.kill_now = True


if __name__ == "__main__":
    rospy.init_node('rssi_recorder')
    rospy.loginfo("Starting rssi recording node")

    # init serial port
    ttyStr = '/dev/localino'
    localino = serial.Serial(ttyStr, 9600)

    # init graceful killer
    gKill = GracefulKiller()

    # lists
    pose_list_x = []
    pose_list_y = []
    pose_list_z = []
    intensity_list = []

    pose = [0,0,0]
    intensity = 92
    # loop
    while not gKill.kill_now:
        
        # get data from localino
        # wait for msg
        # add to list
    
#    newList = list(zip(r.distances, r.meas1, r.meas2))
#    print(newList)

    fileName = typeTest + '_rssi.csv'
    print(fileName)
    with open(fileName, "w+") as data:
        csvWriter = csv.writer(data)
        csvWriter.writerows(newList)
        print("SAVING")

    # can we assume that it's been killed and we're no longer spinning??? or we store it on every final callback)
