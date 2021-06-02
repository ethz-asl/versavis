#!/bin/bash

catkin build versavis
devel=/`catkin config | grep ^"Devel Space:" |  cut -d'/' -f2-`
source_path="$devel"/setup.bash

echo $source_path
source $source_path
rm -rf libraries/ros_lib
rosrun rosserial_arduino make_libraries.py libraries
