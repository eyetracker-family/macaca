#! /bin/sh
# return EETF to ARTagTF
roslaunch handeye_calib_camodocal handeye_tf.launch data_folder:=~/macaca/data/handeye/eyetracker/scene/right ARTagTF:=scene/right/camera_link  cameraTF:=scene/right/marker_ar_0 EETF:=eyetracker_link baseTF:=lighthouse_link

