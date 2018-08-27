#! /bin/sh
# return EETF to ARTagTF
roslaunch handeye_calib_camodocal handeye_tf.launch data_folder:=/home/volcanoh/macaca/data/handeye/eyetracker/scene/left ARTagTF:=scene/left/camera_link cameraTF:=scene/left/ar_marker_0  EETF:=eyetracker_link baseTF:=lighthouse_link

