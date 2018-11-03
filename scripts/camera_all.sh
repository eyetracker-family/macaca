#!/bin/sh

#roslaunch lighthouse eyetracker.launch &

#sleep 2 && roslaunch macaca_tf_setup tf_setup.launch &

roslaunch usbcam usbcam_eye.launch id0:=0 id1:=1& 

sleep 1 && roslaunch usbcam usbcam_scene.launch id0:=5 id1:=4&

sleep 1 && roslaunch eyetracking double_pupiltracking.launch & 

sleep 2 && rviz
