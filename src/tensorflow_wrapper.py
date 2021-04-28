#!/usr/bin/env python3.7

import sys
import numpy as np
import cv2 as cv

import rospy
from sensor_msgs.msg import Image

import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.optimizers import Adam

import pilotnet

"""
SOURCE: https://answers.ros.org/question/350904/cv_bridge-throws-boost-import-error-in-python-3-and-ros-melodic/

note: this version returns RGB, not BGR, as in source (since our image topic uses RGB camera)
"""
def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

class TensorflowWrapper():
    def __init__(self, model_path):
        self.session = tf.compat.v1.keras.backend.get_session()

        with self.session.graph.as_default():
            self.model = load_model(model_path)

    def predict(self, image_msg):
        rgb = imgmsg_to_cv2(image_msg)
        bgr = cv.cvtColor(rgb, cv.COLOR_RGB2BGR)

        img = cv.resize(bgr, (320, 240))
        img_crop = pilotnet.preprocess(img)

        input = np.array([img_crop])

        with self.session.graph.as_default():
            tf.compat.v1.keras.backend.set_session(self.session)
            steering_angle = self.model.predict(input, batch_size=1)[0, 0]

        rospy.loginfo("predicted steering angle: {}".format(steering_angle))
        return steering_angle
