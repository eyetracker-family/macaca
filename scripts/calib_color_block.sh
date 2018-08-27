#!/bin/sh


roslaunch usbcam usbcam_eye.launch id0:=2 id1:=3 & 

sleep 1 && roslaunch usbcam usbcam_scene.launch id0:=0 id1:=1&

sleep 1 && roslaunch eyetracking double_pupiltracking.launch & 

sleep 2 && rviz &

sleep 1 && roslaunch macaca_eyescene_map color_block.launch 
