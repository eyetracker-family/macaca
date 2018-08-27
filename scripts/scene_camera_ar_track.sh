#!/bin/sh

roslaunch lighthouse eyetracker.launch &

sleep 2 && roslaunch macaca_tf_setup tf_setup.launch &

sleep 1 && roslaunch usbcam usbcam_scene.launch id0:=1 id1:=0 &

sleep 2 && ROS_NAMESPACE=/scene/left roslaunch ar_track_alvar pr2_bundle_no_kinect.launch  cam_image_topic:=/scene/left/image_rect_color cam_info_topic:=/scene/left/camera_info  output_frame:=/scene/left/camera_link  marker_size:=5.4  bundle_files:=/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0-8_5.4.xml max_track_error:=0.1  max_new_marker_error:=0.04 &

sleep 2 && ROS_NAMESPACE=/scene/right roslaunch ar_track_alvar pr2_bundle_no_kinect.launch  cam_image_topic:=/scene/right/image_rect_color cam_info_topic:=/scene/right/camera_info  output_frame:=/scene/right/camera_link  marker_size:=5.4  bundle_files:=/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0-8_5.4.xml max_track_error:=0.1  max_new_marker_error:=0.04 &

sleep 2 && rviz
