#!/usr/bin/env python3.7

import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

from tensorflow_wrapper import TensorflowWrapper

class Infer():
    def __init__(self):
        self.model_path = rospy.get_param("~model")
        self.model = TensorflowWrapper(self.model_path)

        self.cam_sub = rospy.Subscriber(rospy.get_param("~center_camera_topic"),
                                        Image,
                                        self.cam_callback,
                                        queue_size=1)

        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"),
                                         AckermannDriveStamped,
                                         queue_size=1)

        self.drive_command = AckermannDriveStamped()
        self.drive_command.header.stamp = rospy.get_rostime()
        self.drive_command.header.frame_id = rospy.get_param("tesse/tesse_ros_bridge/body_frame_id_gt", "base_link") # TODO - check this
        self.drive_command.drive.speed = rospy.get_param("~velocity")
        self.drive_command.drive.acceleration = 0
        self.drive_command.drive.jerk = 0
        self.drive_command.drive.steering_angle = 0
        self.drive_command.drive.steering_angle_velocity = 0

        rospy.loginfo("INITIALIZATION COMPLETE")

    def cam_callback(self, image_msg):
        steering_angle = self.model.predict(image_msg)

        self.drive_command.header.stamp = rospy.get_rostime()
        self.drive_command.drive.steering_angle = steering_angle

        self.drive_pub.publish(self.drive_command)

if __name__ == "__main__":
    rospy.init_node("infer")
    _ = Infer()
    rospy.spin()
