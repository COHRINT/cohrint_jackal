This directory is used for analyzing the rssi model of the localinos

What we want:
     point cloud given by vicon of the xyz position of the localino
     store the corresponding intensity into a csv file

     3D scatter plot of the localino transmit intensity w/ varying grades of intensity

     Test with multiple localinos...
     3 tests here: both vertical, both horizontal, one vertical one horizontal
       With casing && without casing

Testing procedure:
- One localino laying flat or upright
  Has the basic tag program on it and is just pinging
  Make sure with long range accuracy mode
- Another tag with the laptop and me walking around, localizing the wand
  Has the simple receive program on it and is spitting out intensities from the tag
  Subscribes to pose of the wand and saves the corresponding wand data & intensity from localino



