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

syncCode = 's'

testType = 'vert_vert'

# Credits Mayank Jaiswal
class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        
    def exit_gracefully(self,signum, frame):
        self.kill_now = True

def reset_localino():
    self.l.reset_input_buffer()
    self.l.reset_output_buffer()

    # sync with the localino
def sync_localino(loc):
    """ Syncs w/ the localino """
    rospy.loginfo("Attempting to sync with localino")
    while not rospy.is_shutdown():
        reset_localino()
        try: # handle the sigterm exception 
            loc.write(syncCode.encode()) # send start condition
            c = loc.read(1)
            if c == '': # we received nothing
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

def read_packet(loc):
    fp_power = float(loc.read(6).decode('utf-8'))
    rx_power = float(loc.read(6).decode('utf-8'))
    sig_quality = float(loc.read(6).decode('utf-8'))

    return [-fp_power, -rx_power, sig_quality]

def decompose_transform_msg(msg):
    """
    Updates the pose using the transform topic from vicon
    """
    
    # Receive the msg
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    z = msg.transform.translation.z
    
    # xr = msg.transform.rotation.x
    # yr = msg.transform.rotation.y
    # zr = msg.transform.rotation.z
    # wr = msg.transform.rotation.w
    # quat = [xr, yr, zr, wr]
    # (_, _, theta) = tf.transformations.euler_from_quaternion(quat)

    # # Round the positions
    # x = round(x, self.precision)
    # y = round(y, self.precision)
    # t = round(theta, self.precision)
    
    return (x,y,z)
    
        
if __name__ == "__main__":
    rospy.init_node('rssi_recorder')
    rospy.loginfo("Starting rssi recording node")

    # init serial port
    ttyStr = '/dev/localino'
    localino = serial.Serial(ttyStr, 9600)

    topic = '/wand/base_footprint'

    # init graceful killer
    gKill = GracefulKiller()

    # lists
    pose_list_x = []
    pose_list_y = []
    pose_list_z = []
    fp_power_list = []
    rx_power_list = []
    sig_quality_list = []

    pose = [0,0,0]
    intensity = 92

    sync_loclaino(localino)
    packet = [0,0,0] # FP power, rx popwer, signal quality
    # loop
    while not gKill.kill_now:
        # get data from localino        
        (fp, rx, sig_q) = read_packet(localino)
        fp_power_list.append(fp)
        rx_power_list.append(rx)
        sig_quality_list.append(sig_q)
        
        # wait for msg        
        msg = rospy.wait_for_message('')
        (x,y,z) = decompose_transform_msg(msg)
        
        # add to list
        pose_list_x.append(x)
        pose_list_y.append(y)
        pose_list_z.append(z)
    
    newList = list(zip(pose_list_x, pose_list_y, pose_list_z, ))
    print(newList)

    fileName = typeTest + '_rssi.csv'
    print(fileName)
    with open(fileName, "w+") as data:
        csvWriter = csv.writer(data)
        csvWriter.writerows(newList)
        print("SAVING")

    # can we assume that it's been killed and we're no longer spinning??? or we store it on every final callback)
