#!/usr/bin/env python

import rospy
import message_filters

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

import datetime
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from threading import RLock

class RecordTraining:
    def __init__(self):
        # exact synchronization example:
        # http://wiki.ros.org/message_filters
        # https://docs.ros.org/en/api/message_filters/html/python/#message_filters.TimeSynchronizer
        #self.left_sub = message_filters.Subscriber(rospy.get_param("~left_camera_topic"), Image)
        #self.right_sub = message_filters.Subscriber(rospy.get_param("~right_camera_topic"), Image)
        #self.ts = message_filters.TimeSynchronizer([self.left_sub, self.right_sub], queue_size=1)
        #self.ts.registerCallback(self.stereo_callback)

        # approximate synchronization:
        # http://wiki.ros.org/message_filters/ApproximateTime
        # https://docs.ros.org/en/api/message_filters/html/python/#message_filters.ApproximateTimeSynchronizer
        self.camera_sub = message_filters.Subscriber(rospy.get_param("~center_camera_topic"), Image)
        self.drive_sub = message_filters.Subscriber(rospy.get_param("~drive_topic"), AckermannDriveStamped)
        queue_size = rospy.get_param("~queue_size")
        slop = rospy.get_param("~slop")
        self.ats = message_filters.ApproximateTimeSynchronizer([self.camera_sub, self.drive_sub], queue_size=queue_size, slop=slop)
        self.ats.registerCallback(self.train_callback)

        self.bridge = CvBridge()
        self.state_lock = RLock()
        self.csv_name = "/home/racecar/learning_ws/training_data/{}.csv".format(datetime.datetime.now().strftime("%m_%d_%H_%M_%S_%f")[:-3])
        rospy.loginfo("recording training data at {}".format(self.csv_name))

    # example callback for two topics that share exact same timestamps / are already known to be synced
    def stereo_callback(self, left_img, right_img):
        rospy.loginfo("called with stamps - left: {}, right: {}".format(left_img.header.stamp.to_sec(),
                                                                        right_img.header.stamp.to_sec()))

    # example callback for topics that may not necessarily share exact timestamps
    def train_callback(self, rgb_img, drive_cmd):
        rospy.loginfo("camera at time {} sec ({} encoding), drive command at time {} sec (steering angle {}, speed {})".format(
                        rgb_img.header.stamp.to_sec(),
                        rgb_img.encoding,
                        drive_cmd.header.stamp.to_sec(),
                        drive_cmd.drive.steering_angle,
                        drive_cmd.drive.speed))

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_img, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)

        img = cv.resize(cv_image, (320, 240))
        img_name = "/home/racecar/learning_ws/training_data/{}.jpg".format(datetime.datetime.now().strftime("%m_%d_%H_%M_%S_%f")[:-3])
        cv.imwrite(img_name, img, [int(cv.IMWRITE_JPEG_QUALITY), 85]) # TODO check params
        rospy.loginfo("recorded center camera RGB img at {}".format(img_name))

        with self.state_lock:
            # write image filename, current steering angle and speed to training data csv file
            with open(self.csv_name, "a") as fh:
                new_line = "{},{},{}\n".format(img_name, drive_cmd.drive.steering_angle, drive_cmd.drive.speed)
                fh.write(new_line)
                rospy.loginfo("recorded {} at {}".format(new_line, self.csv_name))

if __name__ == "__main__":
    rospy.init_node("train")
    _ = RecordTraining()
    rospy.spin()
