#! /usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from collections import defaultdict
import argparse
import cv2  # NOQA (Must import before importing caffe2 due to bug in cv2)
import glob
import logging
import os
import sys
import time

from caffe2.python import workspace

from core.config import assert_and_infer_cfg
from core.config import cfg
from core.config import merge_cfg_from_file
from utils.io import cache_url
from utils.timer import Timer
import core.test_engine as infer_engine
import datasets.dummy_datasets as dummy_datasets
import utils.c2 as c2_utils
import utils.logging
import utils.vis as vis_utils

import rospy
from std_msgs.msg import String
from macaca_mask_rcnn_msgs.msg import *
import numpy as np
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from eyetracking_msgs.msg import ImagePoint

c2_utils.import_detectron_ops()
# OpenCL may be enabled by default in OpenCV3; disable it because it's not
# thread safe and causes unwanted GPU memory allocations.
cv2.ocl.setUseOpenCL(False)


class Detection:
    def __init__(self):
        self.cfg = rospy.get_param('~cfg', 'e2e_mask_rcnn_R-101-FPN_2x.yaml')
        self.wts = rospy.get_param('~wts', 'model_final.pkl')
        self.train_wts = rospy.get_param('~train_wts', 'R-101.pkl')
        self.confidence = rospy.get_param('~confidence', 0.9)
        self.sub_img_topic = rospy.get_param('~sub_img_topic', "image_rect_color")
        self.frame_rate = rospy.get_param('~frame_rate', 30)

        self.gpu_id = rospy.get_param('~gpu_id', 0)
        self.mask_on = rospy.get_param('~mask_on', True)

        merge_cfg_from_file(self.cfg)
        cfg.NUM_GPUS = 1
        cfg.MODEL.MASK_ON = True
        cfg.TRAIN.WEIGHTS = self.train_wts
        assert_and_infer_cfg()
        utils.logging.setup_logging(__name__)
        self.logger = logging.getLogger(__name__)
	print(self.wts)
        self.model = infer_engine.initialize_model_from_cfg(self.wts, gpu_id=self.gpu_id)
        self.dummy_coco_dataset = dummy_datasets.get_coco_dataset()

        self.pupil_subscriber = rospy.Subscriber('/scene/left/fit_point', ImagePoint, self.pupil_callback, queue_size=1)
        self.image_subscriber = rospy.Subscriber(self.sub_img_topic, Image, self.callback, queue_size=1)
        # self.compressed_image_subscriber = rospy.Subscriber(self.sub_img_topic+"/compressed", CompressedImage, self.compimgcallback, queue_size=1)

        self.pub_bboxes_topic =  rospy.resolve_name(self.sub_img_topic) + '/bboxes'
        print("Mask RCNN Initialized")
        self.bboxes_publisher =  rospy.Publisher(self.pub_bboxes_topic, BBoxDetArray, queue_size=1)
        self.pub_img_topic = self.sub_img_topic + "_detection"
        self.image_publisher = rospy.Publisher(self.pub_img_topic, Image, queue_size=20)
        self.bridge = CvBridge()
        self.last_detect = rospy.Time.now()
        self.fit_point = []

    def pupil_callback(self, data):
        self.fit_point = [data.x, data.y]
        print(data)
    def callback(self, data):
        t = time.time()

        time1 = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*1e-9
        time2 = float(self.last_detect.secs) + float(self.last_detect.nsecs)*1e-9
        if (time1 - time2 < 0.15):
            return
        else:
            self.last_detect = data.header.stamp

        cv_image = self.bridge.imgmsg_to_cv2(data)
        self.imsw, self.bboxes = self.detect(cv_image, self.gpu_id)

        rows, cols, channels = self.imsw.shape
        print (self.fit_point,"xxxx") 
        if len(self.fit_point)==2 and (self.fit_point[0] < cols) and (self.fit_point[0] > 0) and (self.fit_point[1] < rows) and (self.fit_point[1] > 0):
            cv2.circle(self.imsw, (int(self.fit_point[0]), int(self.fit_point[1])), 4, (0,0,255), 8)
        self.bboxes.header= data.header
        self.bboxes_publisher.publish(self.bboxes)
        pub_imgmsg = self.bridge.cv2_to_imgmsg(self.imsw)
        pub_imgmsg.header = data.header;
        self.image_publisher.publish(pub_imgmsg)
        self.logger.info('Callback time: {:.3f}s'.format(time.time() - t))
        return
    def compimgcallback(self, data):
        t = time.time()

        time1 = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*1e-9
        time2 = float(self.last_detect.secs) + float(self.last_detect.nsecs)*1e-9
        if (time1 - time2 < 0.15):
            return
        else:
            self.last_detect = data.header.stamp

        cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        self.imsw, self.bboxes = self.detect(cv_image, 1)
        self.bboxes.header= data.header
        self.bboxes_publisher.publish(self.bboxes)
        pub_imgmsg = self.bridge.cv2_to_imgmsg(self.imsw)
        pub_imgmsg.header = data.header;
        self.image_publisher.publish(pub_imgmsg)
        self.logger.info('Callback time: {:.3f}s'.format(time.time() - t))
        return
        
    def detect(self, im, id):
        timers = defaultdict(Timer)
        t = time.time()
        with c2_utils.NamedCudaScope(id):
             cls_boxes, cls_segms, cls_keyps = infer_engine.im_detect_all(
                 self.model, im, None, timers=timers
             )

        self.logger.info('Inference time: {:.3f}s'.format(time.time() - t))
        for k, v in timers.items():
            self.logger.info(' | {}: {:.3f}s'.format(k, v.average_time))

        imsw = vis_utils.vis_one_image_opencv(
            im,
            cls_boxes,
            cls_segms,
            cls_keyps,
            self.confidence,
            2,
            show_box = True,
            dataset=self.dummy_coco_dataset,
            show_class = True
        )
        boxes, segms, keypoints, classes = vis_utils.convert_from_cls_format(
                cls_boxes,
                cls_segms,
                cls_keyps)
        bboxes = BBoxDetArray()
        bboxes.header = std_msgs.msg.Header()
        if boxes is not None:
            for i in range(len(boxes)):
                box = boxes[i][0:4]
                score = boxes[i][4]
                cls = self.dummy_coco_dataset.classes[classes[i]]
                if (score >= self.confidence):
                    bbox = BBox(box[0], box[1], box[2],box[3])
                    bbox_det = BBoxDet(bbox, score, cls)
                    bboxes.bboxes.append(bbox_det)
            return imsw, bboxes




def main(argv):
    rospy.init_node('macaca_mask_rcnn_node')
    detection = Detection()
    rospy.spin()


if __name__ == '__main__':
    workspace.GlobalInit(['caffe2', '--caffe2_log_level=0'])
    main(sys.argv)
