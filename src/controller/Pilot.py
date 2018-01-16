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
import cv2

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
        self.aps_ts = 0
        self.dvs_ts = 0
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
        self.mode = img_config["mode"]
        print (self.mode)

        # Load Keras Model - Publish topic - CarController
        rospy.init_node("pilot_steering_model", anonymous=True)
        # load keras model from start
        self.joy = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.control_signal = rospy.Publisher('/drive_pwm', Pwm, queue_size=1)

        # subscriber for image and event
        self.camera = rospy.Subscriber(
            "/dvs/image_raw", Image, queue_size=1,
            callback=self.aps_callback)
        self.events = rospy.Subscriber(
            "/dvs/events", EventArray, queue_size=1,
            callback=self.dvs_callback)
        #  self.camera = message_filters.Subscriber(
        #      "/dvs/image_raw", Image, queue_size=1)
        #  self.events = message_filters.Subscriber(
        #      "/dvs/events", EventArray, queue_size=1)
        #  self.ts = message_filters.ApproximateTimeSynchronizer(
        #      [self.camera, self.events], queue_size=2, slop=1)
        #  self.ts.registerCallback(self.test_callback)

        # Lock which waiting for Keras model to make prediction
        rospy.Timer(rospy.Duration(0.005), self.send_control)

    def joy_callback(self, joy):
        global throttle
        throttle = joy.axes[3]  # Let user can manual throttle

    def aps_callback(self, camera_img):
        if self.mode in [1, 2]:
            self.aps_ts = int(camera_img.header.stamp.to_sec()*1e6)
            self.image = cv_bridge.imgmsg_to_cv2(camera_img)
            cv2.imshow("aps", self.image)
            self.image = np.asarray(self.image, dtype=np.float32)
        else:
            self.image = None

        cv2.imshow("dvs", self.event_img/16.)
        cv2.waitKey(1)

        #  if self.model is None:
        #      self.model = self.get_model()

        # do custom image processing here
        input_img = self.img_proc(self.image, self.event_img,
                                  config=self.img_config)
        print ("time diff:", (self.dvs_ts-self.aps_ts)/1e6)

        #  steering, _ = self.predict(self.model, input_img)
        #  print (steering*500+1500)
        self.completed_cycle = True

    def dvs_callback(self, event_list):
        if self.mode in [0, 2]:
            self.dvs_ts = int(event_list.header.stamp.to_sec()*1e6)
            # only process first 1000 events
            event_list = event_list.events
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
        else:
            self.event_img = None

    def callback(self, camera_img, event_list):
        global steering, throttle
        if self.lock.acquire(True):
            # get aps image
            if self.mode in [1, 2]:
                self.image = cv_bridge.imgmsg_to_cv2(camera_img)
                self.image = np.asarray(self.image, dtype=np.float32)
            else:
                self.image = None
            # form dvs image
            if self.mode in [0, 2]:
                event_list = event_list.events
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
            else:
                self.event_img = None

            if self.model is None:
                self.model = self.get_model()
            # do custom image processing here
            input_img = self.img_proc(self.image, self.event_img,
                                      config=self.img_config)

            steering, _ = self.predict(self.model, input_img)
            print (steering*500+1500)
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
