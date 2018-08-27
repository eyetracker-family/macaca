#!/bin/sh
roslaunch usbcam usbcam_eye.launch & 
sleep 2 && ROS_NAMESPACE=/eye/left roslaunch ar_track_alvar pr2_bundle_no_kinect.launch  cam_image_topic:=/eye/left/image_rect_color cam_info_topic:=/eye/left/camera_info  output_frame:=/eye/left/camera_link  marker_size:=0.5  bundle_files:=/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0_1_2_3_4_5_6_7_8.xml max_track_error:=0.1  max_new_marker_error:=0.04 &
sleep 2 && ROS_NAMESPACE=/eye/right roslaunch ar_track_alvar pr2_bundle_no_kinect.launch  cam_image_topic:=/eye/right/image_rect_color cam_info_topic:=/eye/right/camera_info  output_frame:=/eye/right/camera_link  marker_size:=0.5  bundle_files:=/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0_1_2_3_4_5_6_7_8.xml max_track_error:=0.1  max_new_marker_error:=0.04 &
sleep 2 && roslaunch eyetracking double_pupiltracking.launch & 
sleep 2 && rviz
