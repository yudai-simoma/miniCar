#!/usr/bin/env python

import time

import pandas as pd
import numpy as np

import rospy
from std_msgs.msg import Int16, Time
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge, CvBridgeError

from waypoints_finders.finders import find_waypoints, find_angle
from ros_numpy import image, numpify


class DepthController:
    MM_IN_M = 1000

    def __init__(
            self,
            distance_threshold,
            num_angles,
            sphere_radius,
            step_len,
            debug,
            steer_multiplier,
            horizontal_stretch,
    ):
        self.distance_threshold = distance_threshold
        self.num_angles = num_angles
        self.sphere_radius = sphere_radius
        self.step_len = step_len
        self.debug = debug

        self.steer_multiplier = steer_multiplier
        self.horizontal_stretch = horizontal_stretch

        self.pub_servo = rospy.Publisher('/servo', Int16, queue_size=1)
        self.pub_esc = rospy.Publisher('/esc', Int16, queue_size=1)

        self.bridge = CvBridge()

        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self._depth_cb)

        if self.debug:
            self.pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=1)

        self.servo_neutral = 1479
        self.servo_range = 500
        self.esc_neutral = 1500

        self.angle_mult = 600 #80 # 600
        self.throt_mult = 200 # 80 # 200

        rospy.Subscriber('/joy', Joy, self._joy_cb, queue_size=1)

        rospy.init_node('depth_controller')

    def _depth_cb(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float) / self.MM_IN_M

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.choose_angle()

    def choose_angle(self):
        if self.depth_frame is None:
            return

        # Makes the code a bit less cluttered
        depth_frame = self.depth_frame

        depth_frame[depth_frame > self.distance_threshold] = np.nan
        depth_frame[depth_frame == 0] = np.nan

        start_time = time.time()
        waypoints, frame = find_waypoints(
            depth_frame,
            horizontal_stretch=self.horizontal_stretch,
            statistic=np.nanmean,
            num_angles=self.num_angles,
            step_len=self.step_len,
            sphere_radius=self.sphere_radius,
            draw_waypoints=self.debug,
        )

        if self.debug:
            delta = time.time() - start_time
            rospy.loginfo('Finding waypoints took: {:.1f}ms'.format(1000*delta))

            # Also, publish an image for debugging
            depth_frame[np.isnan(depth_frame)] = 0
            depth_frame -= depth_frame.min()
            depth_frame = 127*(depth_frame / depth_frame.max())
            debug_image = image.numpy_to_image(depth_frame.astype('uint8'), encoding='mono8')
            self.pub_debug_image.publish(debug_image)

        start_time = time.time()
        angle = find_angle(waypoints)

        if self.debug:
            delta = time.time() - start_time
            rospy.loginfo('Finding angle took: {:.1f}ms'.format(1000*delta))

        angle = self.servo_neutral + int(self.steer_multiplier*angle*self.servo_range)

        self.pub_servo.publish(angle)


    def _joy_cb(self, msg):
        throt = msg.axes[3]
        throt = self.esc_neutral + int(self.throt_mult*throt)

        self.pub_esc.publish(throt)


if __name__ == '__main__':
    distance_threshold = rospy.get_param('/distance_threshold')
    num_angles = rospy.get_param('/num_angles')
    sphere_radius = rospy.get_param('/sphere_radius')
    step_len = rospy.get_param('/step_len')
    debug = rospy.get_param('/debug')
    steer_multiplier = rospy.get_param('/steer_multiplier')
    horizontal_stretch = rospy.get_param('/horizontal_stretch')

    depth_controller = DepthController(
            distance_threshold,
            num_angles,
            sphere_radius,
            step_len,
            debug,
            steer_multiplier,
            horizontal_stretch,
    )

    depth_controller.spin()
