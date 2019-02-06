#!/usr/bin/env python2

import sys
import serial
import rospy
from gps.msg import *

#####Global Variables######################################
ser = 0

#####FUNCTIONS#############################################
#initialize serial connection 
def init_serial():

	global ser #must be declared in each fxn used
	ser = serial.Serial()
	ser.baudrate = 9600
   
	ser.port = '/dev/ttyACM0' #uncomment for linux

	#you must specify a timeout (in seconds) so that the
	# serial port doesn't hang
	ser.timeout = 1
	ser.open() #open the serial port

	# print port open or closed
	if ser.isOpen():
		print 'Open: ' + ser.port



if __name__ == '__main__':
	rospy.init_node('GPS')

	pub = rospy.Publisher('GPS', Gps, queue_size = 10, latch = True)
	msg = Gps()

	#####SETUP################################################
	init_serial()

	#####MAIN LOOP############################################
	while 1:
		bytes = ser.readline() #reads in bytes 


		if 'GNGGA' in bytes:
			bytes = bytes.split(',')
			lat = (bytes[2])
			lon = (bytes[4])
			if not lon:
				print('No GPS data')
				
			else:
				lat = float(lat)
				lon = float(lon)
				lat_degree = (float(lat) / 100);
				lng_degree = (float(lon) / 100);

				lat_mm_mmmm = lat % 100
				lng_mm_mmmmm = lon % 100


				converted_latitude = lat_degree + (lat_mm_mmmm / 60)
				converted_longitude = lng_degree + (lng_mm_mmmmm / 60)

				print converted_latitude 
				print bytes[3]
				print converted_longitude 
				print bytes[5]
				print bytes[9] #altitude

				msg.latitude = float(converted_latitude)
				msg.longitude = float(converted_longitude)
				msg.altitude = float(bytes[9])
				msg.lat_tag = bytes[3]
				msg.lon_tag = bytes[5]
				msg.stamp = rospy.Time.now()
				pub.publish(msg)
