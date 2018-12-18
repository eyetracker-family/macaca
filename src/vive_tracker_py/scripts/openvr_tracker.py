import triad_openvr
import time
import sys

import roslib
roslib.load_manifest('vive_tracker_py')
import rospy
import tf

import numpy as np

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

#if __name__=='__name__':
rospy.init_node('vive_tracker')
r=rospy.Rate(30)
br=tf.TransformBroadcaster()
while not rospy.is_shutdown():
    r.sleep()

    pose1=v.devices["tracker_3"].get_pose_rotationMatrix()
    pose_mat_np1=np.reshape(pose1,[4,4])
    br.sendTransform((pose1[3],pose1[7],pose1[11]),tf.transformations.quaternion_from_matrix(pose_mat_np1),rospy.Time.now(),"tracker0_link","lighthouse_link")#"tracker1_link"-->"lighthouse_link" c++ is different from python!!!

    pose2=v.devices["tracker_1"].get_pose_rotationMatrix()
    pose_mat_np2=np.reshape(pose2,[4,4])
    br.sendTransform((pose2[3],pose2[7],pose2[11]),tf.transformations.quaternion_from_matrix(pose_mat_np2),rospy.Time.now(),"tracker1_link","lighthouse_link")

    pose3=v.devices["tracker_2"].get_pose_rotationMatrix()
    pose_mat_np3=np.reshape(pose3,[4,4])
    br.sendTransform((pose3[2],pose3[7],pose3[11]),tf.transformations.quaternion_from_matrix(pose_mat_np3),rospy.Time.now(),"tracker2_link","lighthouse_link")

    '''txt1 = ""
    pose1=v.devices["tracker_1"].get_pose_quaternion()
    for each in pose1:#x,y,z,w,r_x,r_y,r_z
        txt1 += "%.4f" % each
        txt1 += " "
    print("tracker_1: "+txt1)
    br.sendTransform((pose1[0],pose1[1],pose1[2]),(pose1[4],pose1[5],pose1[6],pose1[3]),rospy.Time.now(),"tracker0_link","lighthouse_link")#"tracker1_link"-->"lighthouse_link" c++ is different from python!!!

    txt2 = ""
    pose2=v.devices["tracker_2"].get_pose_quaternion()
    for each in pose2:#x,y,z,w,r_x,r_y,r_z
        txt2 += "%.4f" % each
        txt2 += " "
    print("tracker_2: "+txt2)
    br.sendTransform((pose2[0],pose2[1],pose2[2]),(pose2[4],pose2[5],pose2[6],pose2[3]),rospy.Time.now(),"tracker1_link","lighthouse_link")#r_x,r_y,r_z,w

    txt3 = ""
    pose3=v.devices["tracker_3"].get_pose_quaternion()
    for each in pose3:#x,y,z,w,r_x,r_y,r_z
        txt3 += "%.4f" % each
        txt3 += " "
    print("tracker_3: "+txt3)
    br.sendTransform((pose3[0],pose3[1],pose3[2]),(pose3[4],pose3[5],pose3[6],pose3[3]),rospy.Time.now(),"tracker2_link","lighthouse_link")

    txt4 = ""
    pose4=v.devices["tracker_4"].get_pose_quaternion()
    for each in pose4:#x,y,z,w,r_x,r_y,r_z
        txt4 += "%.4f" % each
        txt4 += " "
    print("tracker_4: "+txt4)
    br.sendTransform((pose4[0],pose4[1],pose4[2]),(pose4[4],pose4[5],pose4[6],pose4[3]),rospy.Time.now(),"tracker3_link","lighthouse_link")

    br.sendTransform((0,0,0.2),(0,0,0,1),rospy.Time.now(),"test_link","lighthouse_link")'''
