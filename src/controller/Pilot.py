#!/usr/bin/env python
"""Autopilot mode.

This script activate pilot mode to take control over Jetson Car.

Use X button on joystick to stop
"""
from __future__ import print_function

import threading

import numpy as np
import rospy
import message_filters
from cv_bridge import CvBridge

from sensor_msgs.msg import Joy, Image
from rally_msgs.msg import Pwm
from dvs_msgs.msg import EventArray

steering = 0.0
throttle = 0.0
cv_bridge = CvBridge()

print("Building Pilot Model...")


class Pilot:
    # Activate autonomous mode in Jetson Car
    def __init__(self, get_model_call_back, model_callback,
                 img_proc_callback, img_config=None):
        self.image = None
        self.model = None
        self.event_img = None
        self.get_model = get_model_call_back
        self.predict = model_callback
        self.img_proc = img_proc_callback
        self.completed_cycle = False
        self.start = 0.
        self.lock = threading.RLock()
        # img_config contains info for custom processing for model
        self.img_config = img_config
        self.img_shape = img_config["img_shape"]
        self.histrange = [(0, v) for v in self.img_shape]
        self.clip_value = img_config["clip_value"]

        # Load Keras Model - Publish topic - CarController
        rospy.init_node("pilot_steering_model", anonymous=True)
        # load keras model from start
        self.joy = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.control_signal = rospy.Publisher('/drive_pwm', Pwm, queue_size=1)

        # subscriber for image and event
        self.camera = message_filters.Subscriber(
            "/dvs/image_raw", Image)
        self.events = message_filters.Subscriber(
            "/dvs/events", EventArray)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.camera, self.events], queue_size=1, slop=0.1)
        self.ts.registerCallback(self.callback)

        #  self.camera = rospy.Subscriber(
        #      '/dvs/image_raw', Image, self.callback, queue_size=1)
        # TODO: subscribe DVS event
        #  self.events = rospy.Subscriber(
        #      "/dvs/events", EventArray, self.callback, queue_size=1)

        # Lock which waiting for Keras model to make prediction
        rospy.Timer(rospy.Duration(0.005), self.send_control)

    def joy_callback(self, joy):
        global throttle
        throttle = joy.axes[3]  # Let user can manual throttle

    def callback(self, camera_info):
        global steering, throttle
        if self.lock.acquire(True):
            # get aps image
            self.image = cv_bridge.imgmsg_to_cv2(camera_info[0])
            self.image = np.asarray(self.image, dtype=np.float32)
            # form dvs image
            event_list = camera_info[1]
            event_pol = np.array([x.polarity for x in event_list],
                                 dtype=np.bool)
            event_loc = np.array([[x.x, x.y] for x in event_list],
                                 dtype=np.uint16)

            pol_on = (event_pol[:] == 1)
            pol_off = np.logical_not(pol_on)
            img_on, _, _ = np.histogram2d(
                    event_loc[pol_on, 1], event_loc[pol_on, 0],
                    bins=self.img_shape, range=self.histrange)
            img_off, _, _ = np.histogram2d(
                    event_loc[pol_off, 1], event_loc[pol_off, 0],
                    bins=self.img_shape, range=self.histrange)
            if self.clip_value is not None:
                integrated_img = np.clip(
                    (img_on-img_off), -self.clip_value, self.clip_value)
            else:
                integrated_img = (img_on-img_off)
            self.event_img = integrated_img+self.clip_value

            if self.model is None:
                self.model = self.get_model()
            # do custom image processing here
            input_img = self.img_proc(self.image, self.event_img,
                                      config=self.img_config)

            steering, _ = self.predict(self.model, input_img)
            self.completed_cycle = True
            self.lock.release()

    def send_control(self, event, verbose=False):
        global steering, throttle
        if self.image is None:
            return
        if self.completed_cycle is False:
            return
        # Publish a rc_car_msgs
        msg = Pwm()
        msg.steering = steering
        msg.throttle = throttle
        self.control_signal.publish(msg)
        if verbose is True:
            print("Steering: %.2f. Throttle: %.2f" % (steering, throttle))
        self.completed_cycle = False
