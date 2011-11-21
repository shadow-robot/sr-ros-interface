#!/bin/bash

echo "Regenerating URDF files"
source ~/.bashrc.d/ros.sh
roscd sr_hand/model/robots

for i in $( ls *.urdf ) ; do
    echo "Generating "$i
    `rosrun xacro xacro.py -o $i xacro/$i.xacro` ;
done
