#!/usr/bin/env python

import time

import pandas as pd
import numpy as np

from numba import jit

import rospy
from std_msgs.msg import Int16, Time
from sensor_msgs.msg import Joy, PointCloud2
import sensor_msgs.point_cloud2 as pc2

from utils import pointcloud2_to_array, find_waypoints, find_angle


class PointcloudController:
    def __init__(self, max_angle, num_angles, sphere_radius, nan_coeff):
        rospy.init_node('pointcloud_controller')
        self.max_angle = max_angle
        self.num_angles = num_angles
        self.sphere_radius = sphere_radius
        self.nan_coeff = nan_coeff

        self.pub_servo = rospy.Publisher('/servo', Int16, queue_size=10)
        self.pub_esc = rospy.Publisher('/esc', Int16, queue_size=10)

        self.servo_neutral = 1479
        self.servo_range = 500
        self.esc_neutral = 1500

        self.angle_mult = 600 #80 # 600
        self.throt_mult = 200 # 80 # 200

        self.max_steering_angle = 0.28

        rospy.Subscriber('/joy', Joy, self._joy_cb, queue_size=10)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self._pointcloud_cb, queue_size=10)

        self.colname_map = {0: 'x', 1: 'z', 2: 'y'}

        self.max_0, self.max_2 = 10, 10  # in meters
        self.min_0, self.min_2 = -10, 0.3  # in meters

        # We need the assumption that the center of the
        # grid is exactly in the middle
        assert self.max_0 + self.min_0 == 0

        self.grid_multiplier = 2
        self.shape_x = int(self.grid_multiplier*(self.max_0 - self.min_0))
        self.shape_y = int(self.grid_multiplier*(self.max_2 - self.min_2))

    def _pointcloud_cb(self, msg):
        start_time = time.time()
        dataframe = pd.DataFrame(
            # pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))
            pointcloud2_to_array(msg)
        )

        import pdb; pdb.set_trace()

        time1 = time.time()
        delta = time1 - start_time
        rospy.loginfo('Reading points into dataframe took: {:.1f}ms'.format(1000*delta))

        dataframe = dataframe.rename(self.colname_map, axis=1)
        condition_on_x = (self.min_0 < dataframe['x']) & (dataframe['x'] < self.max_0)
        condition_on_y = (self.min_2 < dataframe['y']) & (dataframe['y'] < self.max_2)
        dataframe = dataframe[condition_on_x & condition_on_y]

        dataframe = (self.grid_multiplier * dataframe).round().astype('int')

        # Minus because the Z axis is pointed into the ground
        avg_height = -dataframe.groupby(['y', 'x'])['z'].mean()
        avg_height = avg_height.unstack()
        time2 = time.time()
        delta = time2 - time1
        rospy.loginfo('Groupby and unstack took: {:.1f}ms'.format(1000*delta))

        try:
            waypoints, frame = find_waypoints(
                avg_height,
                max_angle=self.max_angle,
                num_angles=self.num_angles,
                sphere_radius=self.sphere_radius,
                draw_waypoints=False,
                nan_coeff=self.nan_coeff,
            )
        except KeyError as e:
            rospy.logwarn(str(e))
            return

        time3 = time.time()
        delta = time3 - time2
        rospy.loginfo('Finding waypoints took: {:.1f}ms'.format(1000*delta))

        angle = find_angle(waypoints)
        time4 = time.time()
        delta = time4 - time3
        rospy.loginfo('Finding angle took: {:.1f}ms'.format(1000*delta))

        angle /= self.max_steering_angle
        angle = np.clip(angle, -1, 1)
        angle = self.servo_neutral - int(self.servo_range*angle)

        self.pub_servo.publish(angle)


    def _joy_cb(self, msg):
        throt = msg.axes[3]
        throt = self.esc_neutral + int(self.throt_mult*throt)

        self.pub_esc.publish(throt)


if __name__ == '__main__':
    max_angle = rospy.get_param('/max_angle')
    num_angles = rospy.get_param('/num_angles')
    sphere_radius = rospy.get_param('/sphere_radius')
    nan_coeff = rospy.get_param('/nan_coeff')
    PointcloudController(max_angle, num_angles, sphere_radius, nan_coeff)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('PointcloudController failed')
