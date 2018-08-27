#! /bin/sh
# return EETF to ARTagTF
roslaunch handeye_calib_camodocal handeye_tf.launch data_folder:=~/macaca/data/handeye/eyetracker/eye/left  ARTagTF:=eye/left/camera_link  cameraTF:=eye/left/ar_marker_0 EETF:=eyetracker_link  baseTF:=lighthouse_link

