#!/bin/sh

#roslaunch lighthouse eyetracker.launch &

#sleep 2 && roslaunch macaca_tf_setup tf_setup.launch &

roslaunch usbcam usbcam_eye.launch id0:=1 id1:=0& 

sleep 1 && roslaunch usbcam usbcam_scene.launch id0:=5 id1:=4&

sleep 1 && roslaunch eyetracking double_pupiltracking.launch & 

sleep 2 && rviz
