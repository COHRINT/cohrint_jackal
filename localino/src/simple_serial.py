#!/usr/bin/python3

"""

This file develops a simple protocol for consistent serial communication with the localinos.

"""

__author__ = "LT"
__copyright__ = "Copyright 2018, Cohrint"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "LT"
__email__ = "luke.barbier@colorado.edu"
__status__ = "Development"

import serial
import signal
import sys

"""
I should separate the localino node with the rosnode, the rosnode makes a localino object

The localino that I flash knows its number
"""

ttyStr = '/dev/localino' # udev rule (67-localino-rules) loads this
syncCode = 's'

class Localino():
    """
    @param
    letter, string corresponding to the letter name of the localino
    """
    def __init__(self):

        """ Initialize the localino """
                
        self.l = serial.Serial(ttyStr, 9600, timeout=0.25)

        while True:
            self.l.reset_input_buffer()
            self.l.reset_output_buffer()
            try: # handle the sigterm exception 
                self.l.write(syncCode.encode()) # send start condition
                c = self.l.read(1) # block on receiving a char
                ck = c.decode('utf-8') # decode the char
                if ck == syncCode:
                    break
                else:
                    print("other char")
            except Exception as err:
                print(err)
                
        self.l.timeout = None # set the read timeout back to blocking

if __name__ == "__main__":
    Localino()
    print("Successful")
