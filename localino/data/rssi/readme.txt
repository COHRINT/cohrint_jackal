This directory is used for analyzing the rssi model of the localinos

Testing procedure:

In describing the testing procedure, reference the naming of a csv file as OriginLocalinoOrientation_MovingLocalinoOrientation_rssi.csv


- First localino at the origin in OriginLocalinoOrientation with onboard firmware found in arduino/BasicSender/BasicSender.ino
- Other localino laying laying MovingLocalinoOrientation attached to the centroid of the vicon wand moving about the space.

In the CSV files, the recorded data organized by columns are as follows:
1) x location of wand
2) y location
3) z location
4) First Path Power
5) RX Power
6) Signal Quality

A simple matlab file (analyzeRssi.m) that dispalys signal quality and rx power in 3D space is also included. 
