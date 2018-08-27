#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from visp_hand2eye_calibration import msg as vm
import geometry_msgs.msg as geomsg
import numpy as np
import cv2
from tf.transformations import quaternion_from_euler



def read_transforms(file='1.yml'):
    fs = cv2.FileStorage(file, cv2.FILE_STORAGE_READ)
    frameCount = int(fs.getNode('frameCount').real())
    A = []
    B = []
    for i in range(frameCount-1):
        A.append(fs.getNode("T" + "1_" + str(i)).mat())
        B.append(fs.getNode("T" + "2_" + str(i)).mat())
    return A, B


if __name__ == '__main__':
    rospy.init_node('pub_handeye_tf')
    A, B = read_transforms('1.yml')
    Aarr = []
    Barr = []
    for m in A:
        rvec, tvec = cv2.Rodrigues(m[0:3,0:3])[0], m[0:3,3:4]
        quat = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
        tran = geomsg.Transform(tvec, quat)
        Aarr.append(tran)
    for m in B:
        rvec, tvec = cv2.Rodrigues(m[0:3,0:3])[0], m[0:3,3:4]
        quat = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
        tran = geomsg.Transform(tvec, quat)
        Barr.append(tran)

    Amsg = vm.TransformArray()
    Amsg.transforms = Aarr
    Amsg.header = Header()
    Bmsg = vm.TransformArray()
    Bmsg.transforms = Barr
    Bmsg.header = Header()

    # print Amsg

    # pubA = rospy.Publisher('world_effector', vm.TransformArray, queue_size=5)
    pubA = rospy.Publisher('world_effector', geomsg.Transform, queue_size=5)
    # pubB = rospy.Publisher('camera_object', vm.TransformArray, queue_size=5)
    pubB = rospy.Publisher('camera_object', geomsg.Transform, queue_size=5)
    rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    for i in range(5):
        Amsg.header.stamp = rospy.rostime.get_time()
        Bmsg.header.stamp = rospy.rostime.get_time()
        # pubA.publish(Amsg)
        print Amsg.transforms[i]
        pubA.publish(Amsg.transforms[i])
        pubB.publish(Bmsg.transforms[i])
        # pubB.publish(Bmsg)
        rate.sleep()
    # pose = geomsg.Pose(1,2,3,0)
    # geomsg.Quaternion()
    # print t







