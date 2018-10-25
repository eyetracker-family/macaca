#! /usr/bin/env python
import sys
import yaml
from yaml import load, dump
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
import numpy as np
import rospy
from eyetracking_msgs.msg import RotatedRect
from eyetracking_msgs.msg import ImagePoint
import pandas as pd

import cv2  # NOQA (Must import before importing caffe2 due to bug in cv2)
import glob
import logging
import os
import time
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from eyetracking_msgs.msg import ImagePoint


left_eye_point2d_list = []
right_eye_point2d_list = []
left_scene_point2d_list = []

left_eye_point2d_list = []
right_eye_point2d_list = []
left_scene_point2d_list = []

lspt = ImagePoint()

calibrated = False
le_done = False
re_done = False
ls_done = False
lex,ley, rex, rey = 0.0, 0.0, 0.0, 0.0

def read_calib_data(filename):
    global left_scene_point2d_list, left_eye_point2d_list, right_eye_point2d_list
    with open(filename) as stream:
        for i in [0,1]:
            _ = stream.readline()
        data = yaml.load(stream)

    count = data['count']
    del left_eye_point2d_list[:]#delete all the data in the list
    del right_eye_point2d_list[:]
    del left_scene_point2d_list[:]

    for i in range(count):
        left_eye_point2d = data['left_eye_point2d_' + str(i)]
        right_eye_point2d = data['right_eye_point2d_' + str(i)]
        left_scene_point2d = data['left_scene_point2d_' + str(i)]
        left_eye_point2d_list.append(left_eye_point2d)
        right_eye_point2d_list.append(right_eye_point2d)
        left_scene_point2d_list.append(left_scene_point2d)
    # print count

length_scale = [100, 100, 100, 100]#3 originally
length_scale_bounds = [(1e-05, 100000.0), (1e-05, 100000.0), (1e-05, 100000.0), (1e-05, 100000.0)]

gp_kernel_lsx = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)
gp_kernel_lsy = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)
#gp_kernel_rsx = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)
#gp_kernel_rsy = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)

gpr_lsx = GaussianProcessRegressor(kernel=gp_kernel_lsx)
gpr_lsy = GaussianProcessRegressor(kernel=gp_kernel_lsy)
#gpr_rsx = GaussianProcessRegressor(kernel=gp_kernel_rsx)
#gpr_rsy = GaussianProcessRegressor(kernel=gp_kernel_rsy)

def gaze_estimation_model_fitting():
    print("gaze estimation model fitting")
    gpr_lsx.fit(np.concatenate((left_eye_point2d_array, right_eye_point2d_array), axis=1), left_scene_point2d_array[:,0])
    gpr_lsy.fit(np.concatenate((left_eye_point2d_array, right_eye_point2d_array), axis=1), left_scene_point2d_array[:,1])
    print("fitted")


def handle_gaze_estimation():
    global lex,ley,rex,rey
    print ("input", np.reshape(np.array([lex, ley, rex, rey]),[1,4]))

    lsx = gpr_lsx.predict(np.reshape(np.array([lex, ley, rex, rey]),[1,4]), return_std=False)
    lsy = gpr_lsy.predict(np.reshape(np.array([lex, ley, rex, rey]),[1,4]), return_std=False)

    #lsx = gpr_lsx.predict(np.reshape(np.array([687.50, 264.7, 540.9, 308.34]),[1,4]), return_std=False)#556
    #lsy = gpr_lsy.predict(np.reshape(np.array([687.50, 264.7, 540.9, 308.34]),[1,4]), return_std=False)#307
    return lsx, lsy


def leftCallback(data):
    global lex, ley, le_done
    lex = data.x
    ley = data.y
    le_done = True

def rightCallback(data):
    global rex, rey, re_done
    rex = data.x
    rey = data.y
    re_done = True

def imageCallback(data):
    global image,ls_done
    image=CvBridge().imgmsg_to_cv2(data)
    ls_done=True

if __name__ == '__main__':
    rospy.init_node('fit_point')

    left_eye_sub = rospy.Subscriber('/eye/left/pupil_ellipse', RotatedRect, leftCallback, queue_size=1)
    right_eye_sub = rospy.Subscriber('/eye/right/pupil_ellipse', RotatedRect, rightCallback, queue_size=1)
    left_scene_pub = rospy.Publisher('/scene/left/fit_point', ImagePoint, queue_size=1)

    left_image_sub = rospy.Subscriber('/scene/left/image_color', Image, imageCallback, queue_size=1)
    #left_image_pub = rospy.Publisher('/scene/left/image_gaze_point', Image, queue_size=20)#left image with gaze point
    r = rospy.Rate(30)
    calib_data_file = rospy.get_param('/fit_point/calib_data')
    while not rospy.is_shutdown():
        # read calibration data
        r.sleep()
        #rospy.spinOnce()
        if not calibrated:
            read_calib_data(calib_data_file)
            left_eye_point2d_array = np.array(left_eye_point2d_list)
            right_eye_point2d_array = np.array(right_eye_point2d_list)
            left_scene_point2d_array = np.array(left_scene_point2d_list)
            gaze_estimation_model_fitting()
            calibrated = True
            # get fitting model with data
        else:
            if le_done and re_done and ls_done:
                lsx, lsy= handle_gaze_estimation()
                #lspt = ImagePoint()
                lspt.x = lsx
                lspt.y = lsy
                left_scene_pub.publish(lspt)
                #cv2.circle(image, (int(lspt.x), int(lspt.y)), 8, (0,0,255), 8)
                #pub_imgmsg = CvBridge().cv2_to_imgmsg(image)
                #left_image_pub.publish(pub_imgmsg)
                #cv2.imshow("image",image)
                #cv2.waitKey(0)
                # print(lspt)
                print(lsx,lsy)
    print("shutdown")
    rospy.spin()
