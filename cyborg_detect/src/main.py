#!/usr/bin/env python

import os
import pyyolo
import rospy
import roslib
import rospkg
import sys

import cv2 as cv
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from cyborg_detect.cfg import DetectorConfig
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Point32
from cyborg_detect.msg import Prediction, Predictions
from sensor_msgs.msg import Image

NAME = 'cyborg_detect'


class Pred(object):
    def __init__(self, label, confidence, box, distance=0):
        self.label = label
        self.confidence = confidence
        self.box = box
        self.distance = distance

    def draw(self, frame):
        """ Draw the prediction on the provided frame """
        left, right, top, bottom = self.box['left'], self.box['right'], self.box['top'], self.box['bottom']
        text = '{}: {:.2f} ({:.2f} m)'.format(self.label, self.confidence, self.distance)

        # Draw label
        text_size, baseline = cv.getTextSize(text, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, text_size[1])

        cv.rectangle(frame, (left, top - text_size[1]), (left + text_size[0], top + baseline), (255, 255, 255), cv.FILLED)
        cv.putText(frame, text, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

        # Draw bounding box
        cv.rectangle(frame, (left, top), (right, bottom), (0, 255, 0))

    def to_msg(self):
        """ Convert the Prediction to an ROS message """
        msg = Prediction()
        prediction.label = self.label
        prediction.confidence = self.confidence
        prediction.bounding_box.points = [
                Point32(x=self.box['left'], y=self.box['top']),
                Point32(self.box['right'] + self.box['top']),
                Point32(self.box['right'] + self.box['bottom']),
                Point32(self.box['left'] + self.box['bottom'])]
        prediction.distance = self.distance

        return msg


class PredictionContainer(object):
    def __init__(self, image_header, data, depth=None):
        self.image_header = image_header
        self.predictions = []

        for res in data:
            label = res['class']
            confidence = res['prob']
            box = {'left': res['left'], 'right': res['right'], 'top': res['top'], 'bottom': res['bottom']}
            distance = self.__calculate_distance(box, depth) if depth is not None else 0

            self.predictions.append(Pred(label, confidence, box, distance))

    @staticmethod
    def __calculate_distance(box, depth):
        l = box['left']
        r = box['right']
        t = box['top']
        b = box['bottom']

        distance = np.nanmedian(depth[l:r, t:b].astype(np.float32))

        if np.isnan(distance):
            center = (r - (r - l) // 2, b - (b - t) // 2)
            distance = depth[center[1], center[0]] 

        return distance

    def draw(self, frame):
        """ Draw all predictions on the provided frame """
        for prediction in self.predictions:
            prediction.draw(frame)

    def to_msg(self):
        """ Convert all predictions to an ROS message """
        msg = Predictions()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.image_header = self.image_header
        msg.predictions = [prediction.to_msg() for prediction in self.predictions]

        return msg


class Detector(object):
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)

        # Image detection parameters
        self.conf_thresh = 0.6
        self.hier_thresh = 0.8
        self.frame_height = 376
        self.frame_width = 672

        # Set up dynamic reconfigure
        self.srv = Server(DetectorConfig, self.reconf_cb)

        # Set up OpenCV Bridge
        self.bridge = CvBridge()

        # Set up detection image publisher
        det_topic = rospy.get_param('~detection_image_topic_name')
        self.det_pub = rospy.Publisher(det_topic, Image, queue_size=1, latch=True)

        # Set up prediction publisher
        pred_topic = rospy.get_param('~predictions_topic_name')
        self.pred_pub = rospy.Publisher(pred_topic, Predictions, queue_size=1, latch=True)

        # The first subscriber retrieves image data from the left image and passes it to the callback function
        image_topic = rospy.get_param('~camera_topic_name') 
        image_sub = Subscriber(image_topic, Image, queue_size=1)

        # The second subscriber retrieves depth data from the camera as 32 bit floats with values in meters and maps this onto an image structure, which is passed to callback2
        depth_topic = rospy.get_param('~depth_topic_name') 
        depth_sub = Subscriber(depth_topic, Image, queue_size=1)

        # We want to receive both images in the same callback
        ts = ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=1, slop=0.1)
        ts.registerCallback(self.image_cb)

    def __prepare_image(self, image_msg):
        data = (np.fromstring(image_msg.data, np.uint8)
                .reshape((self.frame_height, self.frame_width, 3))
                .transpose(2, 0, 1)
                .ravel() / 255.0)
        return np.ascontiguousarray(data, dtype=np.float32)

    def image_cb(self, image_msg, depth_msg):
        rospy.loginfo('Received images')

        det_conns = self.det_pub.get_num_connections()
        pred_conns = self.pred_pub.get_num_connections()

        # Early return if no one is listening
        if not det_conns and not pred_conns:
            return

        # The retrieved image in ros_data is reshaped from a flat structure and normalized
        data = self.__prepare_image(image_msg)
        output = pyyolo.detect(self.frame_width, self.frame_height, 3, data, self.conf_thresh, self.hier_thresh)

        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        predictions = PredictionContainer(output, image_msg.header, depth)

        if det_conns:
            predictions.draw(frame)
            
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            self.det_pub.publish(image_msg)

        if pred_conns:
            pred_msg = predictions.to_msg()
            self.pred_pub.publish(pred_msg)

    def reconf_cb(self, config, level):
        rospy.loginfo('Reconfigure request')
        self.conf_thresh = config['conf_threshold']

        return config


def main(args):
    rp = rospkg.RosPack()

    darknet_path = '/home/nvidia/pyyolo/darknet'
    datacfg = os.path.join(rp.get_path(NAME), 'data/cfg', 'coco.data')
    cfgfile = os.path.join(rp.get_path(NAME), 'data/cfg', 'yolov2.cfg')
    weightfile = os.path.join(rp.get_path(NAME), 'data', 'yolov2.weights')

    pyyolo.init(darknet_path, datacfg, cfgfile, weightfile)
    
    '''Initializes and cleanup ros node'''
    ir = Detector()	

    try:
        rospy.spin()
    except KeyboardInterrupt: 
        print "Shutting down ROS Image feature detector module"
    
    # free model
    pyyolo.cleanup()


if __name__ == "__main__":
    main(sys.argv)
