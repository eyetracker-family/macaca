#!/bin/sh

roslaunch usbcam usbcam_scene.launch id0:=1 id1:=0&

sleep 2 && rviz
