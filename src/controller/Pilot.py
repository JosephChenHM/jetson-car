#!/usr/bin/env python
"""Autopilot mode.

This script activate pilot mode to take control over Jetson Car.

Use X button on joystick to stop
"""
from __future__ import print_function

import threading

import numpy as np
import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Joy, Image
from rally_msgs.msg import Pwm

steering = 0.0
throttle = 0.0
cv_bridge = CvBridge()

print("Building Pilot Model...")


class Pilot:
    # Activate autonomous mode in Jetson Car
    def __init__(self, get_model_call_back, model_callback):
        self.image = None
        self.model = None
        self.get_model = get_model_call_back
        self.predict = model_callback
        self.completed_cycle = False
        self.start = 0.
        self.lock = threading.RLock()

        # Load Keras Model - Publish topic - CarController
        rospy.init_node("pilot_steering_model", anonymous=True)
        # load keras model from start
        if self.model is None:
            self.model = self.get_model()
        self.joy = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.control_signal = rospy.Publisher('/drive_pwm', Pwm, queue_size=1)
        self.camera = rospy.Subscriber(
            '/dvs/image_raw', Image, self.callback, queue_size=1)
        # TODO: subscribe DVS event

        # Lock which waiting for Keras model to make prediction
        rospy.Timer(rospy.Duration(0.005), self.send_control)

    def joy_callback(self, joy):
        global throttle
        throttle = joy.axes[3]  # Let user can manual throttle

    def callback(self, camera):
        global steering, throttle
        if self.lock.acquire(True):
            # get image
            self.image = cv_bridge.imgmsg_to_cv2(camera)
            self.image = np.asarray(self.image, dtype=np.float32)
            # TODO: maybe do event processing here
            steering, _ = self.predict(self.model, self.image)
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
