"""

This class reads in data from the RangingAnchor.ino file and stores the data when the program is killed


Credits Mayank Jaiswal for the GracefulKiller class

TODO write a udev rule for the localinos

"""

import serial
import signal
import time
import numpy
import csv
from pdb import set_trace

class Gatherer():

    dists = []
    times = []
    start_time = time.time()
    
    def __init__(self, ttyStr='/dev/ttyACM0'):
        self.l = serial.Serial(ttyStr, 115200)
        time.sleep(1)
#        self.l.write(chr(s))
        
    def read_distance(self):
        self.times.append(time.time() - self.start_time)
        try:
            
            s = self.l.read(4)
            f = float(s.decode("utf-8"))
            #        set_trace()
            self.dists.append(f)
        except:
            pass

# Credits Mayank Jaiswal
class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        
    def exit_gracefully(self,signum, frame):
        self.kill_now = True

if __name__ == '__main__':
    print("Starting Gatherer")
    killer = GracefulKiller()
    g = Gatherer()
    while True:
        try:
            c = g.l.read(1)
            if ord(c) == ord('#'): # ignore it if it's not the case
                g.read_distance()
        except serial.serialutil.SerialException:
            pass
        if killer.kill_now:
            break
        
    new_list = list(zip(g.dists, g.times))
#    set_trace()
    with open("localino_range_data.csv", "w+") as data:
        csvWriter = csv.writer(data)
        csvWriter.writerows(new_list)
        print("saving")
    print("Test Done")
