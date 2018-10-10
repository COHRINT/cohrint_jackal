This directory contains data taken from localino measurements in different combinations of localinos. Each localino has a name printed on the back side of the board. For each csv file name1_name2.csv, the columns are as follows:

1) True distance in [m]
2) name1's measurement of the distance from name1 to name2
3) name2's measurement of the distance from name2 to name1

A matlab script (localinoAnalysis.m) combines all of the data and plots the raw data and error as functions of distance.
