#! /bin/sh
# return EETF to ARTagTF
roslaunch handeye_calib_camodocal handeye_tf.launch data_folder:=/home/volcanoh/macaca/data/camera/scene/stereo ARTagTF:=scene/left/camera_link cameraTF:=scene/left/ar_marker_0  EETF:=scene/right/camera_link baseTF:=scene/right/ar_marker_0

