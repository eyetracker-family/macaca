import triad_openvr
import time
import sys

import roslib
roslib.load_manifest('vive_tracker_py')
import rospy
import tf

import numpy as np

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#global l_image,r_image

def leftCallback(data):
    global l_image
    l_image=CvBridge().imgmsg_to_cv2(data)

def rightCallback(data):
    global r_image
    r_image=CvBridge().imgmsg_to_cv2(data)

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

count=0
file=open('/home/macaca/macaca/data/axyb/eyetracker2lighthouse.txt','w')  

#if __name__=='__name__':
rospy.init_node('vive_tracker')
r=rospy.Rate(30)

left_eye_sub = rospy.Subscriber('/scene/left/image_raw', Image, leftCallback, queue_size=1)
right_eye_sub = rospy.Subscriber('/scene/right/image_raw', Image, rightCallback, queue_size=1)

while not rospy.is_shutdown():
    r.sleep()
    #if(l_image.Empty()|r_image.Empty()):
        #continue

    pose1=v.devices["tracker_1"].get_pose_rotationMatrix()
    cv2.imshow('left',l_image)
    cv2.imshow('right',r_image)
    c=cv2.waitKey(2)
    if(c==115):#S
        count+=1
        filename='/home/macaca/macaca/data/axyb/'
        l_filename=filename+'left'+str(count)+'.jpg'
        r_filename=filename+'right'+str(count)+'.jpg'
        cv2.imwrite(l_filename,l_image)
        cv2.imwrite(r_filename,r_image)
        file.write(str(count)+': '+str(pose1)+'\n');
file.close()
        

