#!/usr/bin/env python
"""Main controller.

This script is used to start/stop record rosbag using Joy ROS message
"""
from __future__ import print_function
import rospy
#  import roslib
import subprocess
import os
import signal
from sensor_msgs.msg import Joy


def callback(joy):
    global autonomous, activate_pilot

    # RIGHT TOP - BIG BUTTON - RT
    if joy.buttons[7] == 1:
        if autonomous is False:
            autonomous = True
            activate_pilot = subprocess.Popen(
                'roslaunch controller autopilot.launch',
                stdin=subprocess.PIPE, shell=True)
            print("\n\n", '-'*30)
            print("Starting Autonomous Mode.")
            print('-'*30, "\n")
        else:
            print("\n\n", '-'*30)
            print("Autonomous has been activated already.")
            print('-'*30, "\n")

    # RIGHT TOP - SMALL BUTTON - RB
    if joy.buttons[5] == 1:
        if autonomous is True:
            autonomous = False
            terminate_process_and_children(activate_pilot)
            print("\n\n", '-'*30)
            print("Turning off Autonomous...")
            print('-'*30, "\n")
        else:
            print("\n\n", '-'*30)
            print("Autopilot is not activated yet..."
                  "Press RightTop button to activate.")
            print('-'*30, "\n")


def terminate_process_and_children(p):
    ps_command = subprocess.Popen(
        "ps -o pid --ppid %d --noheaders" % p.pid,
        shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
        os.kill(int(pid_str), signal.SIGINT)
    p.terminate()


def start():
    rospy.init_node("autonomous_controller", anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()


if __name__ == "__main__":
    autonomous = False
    start()
