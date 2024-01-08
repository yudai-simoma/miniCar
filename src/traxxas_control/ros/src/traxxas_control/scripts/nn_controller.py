#!/usr/bin/env python

import os
import time

import pandas as pd
import numpy as np
from keras.models import load_model, Model
from keras.activations import relu

import rospy
from std_msgs.msg import Int16, Time
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge, CvBridgeError

from ros_numpy import image, numpify


def lrelu(x):
    return relu(x, alpha=0.3)


class NeuralNetworkController:
    MM_IN_M = 1000
    decimation = 10

    def __init__(
            self,
            debug,
            steer_multiplier,
            model_path,
            which_future_pred,
            manual_throttle,
            bottom,
            slope,
            boost,
    ):
        if 'log1p' in model_path:
            self.norm_fn = lambda x: -np.log1p(x)
        elif 'identity' in model_path:
            self.norm_fn = lambda x: x
        else:
            raise ValueError('The `model_path` argument should contain the norm function (either "log1p" or "identity")')

        self.steer_multiplier = steer_multiplier
        self.manual_throttle = manual_throttle
        self.go_flag = False
        self.reverse = False

        self.bottom = bottom
        self.slope = slope
        self.boost = boost

        start_time = time.time()
        self.model = self.init_model(model_path, which_future_pred)
        delta = time.time() - start_time
        rospy.loginfo('Success! It took: {:.2f}s'.format(delta))

        self.debug = debug

        self.depth_frame = None
        self.depth_frame_processed = None

        self.pub_servo = rospy.Publisher('/servo', Int16, queue_size=1)
        self.pub_esc = rospy.Publisher('/esc', Int16, queue_size=1)

        self.bridge = CvBridge()
        rospy.Subscriber(
                '/camera/aligned_depth_to_color/image_raw',
                Image,
                self._depth_cb,
                queue_size=1,
                buff_size=16777216,
        )

        if self.debug:
            self.pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=1)

        self.servo_neutral = 1479
        self.servo_range = 400
        self.esc_neutral = 1500
        self.esc_break = 1200
        self.throt = 0

        self.joy_msg = None
        rospy.Subscriber('/joy', Joy, self._joy_cb, queue_size=1)

        rospy.init_node('nn_controller')
        self.rate = rospy.Rate(100)


    def init_model(self, model_path, which_future_pred):
        model_dir = '/'.join(model_path.split('/')[:-1])
        model_filename = model_path.split('/')[-1]
        model_filename = model_filename.replace('.h5', '__{}.h5'.format(which_future_pred))
        submodel_path = model_dir + '/' + model_filename

        if os.path.exists(submodel_path):
            rospy.loginfo('Loading model {}\n...\n...'.format(submodel_path))
            return load_model(submodel_path, custom_objects={'lrelu': lrelu})

        rospy.loginfo('Loading model {}\n...\n...'.format(model_path))
        whole_model = load_model(model_path, custom_objects={'lrelu': lrelu})
        inp = whole_model.layers[0].output

        num_steps_into_future = 1
        for layer in whole_model.layers:
            layer_name = layer.name
            if 'steer__' in layer_name:
                # `layer_name` is, for example, 'steer__10__last'
                steps_into_future = int(layer_name.split('__')[1])
                if steps_into_future > num_steps_into_future:
                    num_steps_into_future = steps_into_future

        which_future_pred = 0
        if which_future_pred == 0:
            output_layer_name = 'steer'
        elif which_future_pred <= num_steps_into_future:
            output_layer_name = 'steer__{}__last'.format(which_future_pred)
        else:
            raise ValueError(
                'Argument `which_future_pred` needs to be <= {}'
                .format(num_steps_into_future)
            )

        out = None
        for layer in whole_model.layers:
            if layer.name == output_layer_name:
                out = layer.output

        model = Model(inp, out)
        # model._make_predict_function()
        model.save(submodel_path)

        return model

    def _depth_cb(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def spin(self):
        while not rospy.is_shutdown():
            angle = self.make_inference()
            if angle:
                self.pub_servo.publish(angle)

            throt = self.calc_throt()
            if throt:
                self.pub_esc(throt)

    def make_inference(self):
        if self.depth_frame is None:
            return

        start_time = time.time()
        height, width = self.depth_frame.shape
        row_offset = (height % self.decimation) // 2
        col_offset = (width % self.decimation) // 2
        depth_frame = self.depth_frame[
                row_offset:-(row_offset+1):self.decimation,
                col_offset:-(col_offset+1):self.decimation
        ]
        self.depth_frame_processed = self.norm_fn(depth_frame.astype(np.float32) / self.MM_IN_M)
        extended_depth_frame = self.depth_frame_processed[np.newaxis, ..., np.newaxis]

        if self.debug:
            delta = time.time() - start_time
            rospy.loginfo('Image preparation took: {:.1f}ms'.format(1000*delta))

        start_time = time.time()
        pred = self.model.predict(extended_depth_frame)[0, 0]
        if self.debug:
            delta = time.time() - start_time
            rospy.loginfo('Prediction took: {:.2f}ms'.format(1000*delta))
            rospy.loginfo('Steer: {:.2f}'.format(pred))

        angle = self.servo_neutral + int(self.steer_multiplier*pred*self.servo_range)

        return angle

    def _joy_cb(self, msg):
        self.joy_msg = msg


    def calc_throt(self):
        if self.joy_msg is None:
            return

        if self.go_flag is False:
            if self.joy_msg.buttons[1] == 1:
                self.go_flag = True
                self.reverse = False
        else:
            if self.joy_msg.buttons[2] == 1 or self.joy_msg.buttons[0] == 1:
                self.go_flag = False
                if self.joy_msg.buttons[0] == 1:
                    self.reverse = True

        if self.manual_throttle:
            throt_inc = self.joy_msg.buttons[5]
            throt_dec = self.joy_msg.buttons[7]
            if throt_inc:
                self.throt += 1
                rospy.logwarn('self.throt = {}'.format(self.throt))
                self.rate.sleep()
            if throt_dec:
                self.throt -= 1
                rospy.logwarn('self.throt = {}'.format(self.throt))
                self.rate.sleep()

            throt = self.esc_neutral + self.throt
        else:
            raise NotImplementedError()
            # WIP: instead of taking a prediction from NN, calculate acceleration
            #  according to the distance in a patch in the center of the depth map
            box_size = 0.1
            shape0 = int(box_size*self.depth_frame_processed.shape[0])
            shape1 = int(box_size*self.depth_frame_processed.shape[1])
            offset0 = (self.depth_frame_processed.shape[0] - shape0) // 2
            offset1 = (self.depth_frame_processed.shape[1] - shape1) // 2
            slice0 = slice(offset0, offset0+shape0)
            slice1 = slice(offset1, offset1+shape1)
            if self.depth_frame_processed is None:
                throt = self.esc_neutral
            else:
                subframe = self.depth_frame_processed[slice0, slice1].astype(np.float)
                rospy.loginfo(subframe)
                subframe[subframe == 0] = np.isnan
                mean = np.nanmean(subframe)
                throt = self.bottom + self.boost / (1 + np.exp(-self.slope * (mean - 4)))

        if self.go_flag is False:
            throt = self.esc_neutral
            if self.reverse is True:
                throt = self.esc_break

        if throt > 2000:
            throt = 2000

        self.pub_esc.publish(throt)


if __name__ == '__main__':
    steer_multiplier = rospy.get_param('/steer_multiplier')
    debug = rospy.get_param('/debug')
    model_path = rospy.get_param('/model_path')
    which_future_pred = rospy.get_param('/which_future_pred')
    manual_throttle = rospy.get_param('/manual_throttle')
    bottom = rospy.get_param('/bottom')
    slope = rospy.get_param('/slope')
    boost = rospy.get_param('/boost')

    nn_controller = NeuralNetworkController(
            debug,
            steer_multiplier,
            model_path,
            which_future_pred,
            manual_throttle,
            bottom,
            slope,
            boost,
    )

    nn_controller.spin()
