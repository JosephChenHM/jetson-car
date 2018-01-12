Jetson Autonomous RC Car
========================

## Setup step

0. Clone the repo to some where in your system.

1. Install catkin tools

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```

The catkin tools will be installed with ROS distribution.

2. Configure catkin tools

```
$ cd $HOME/catkin_ws
$ catkin init
$ catkin clean  # optional if there is no other packages
$ catkin config --merge-devel
```

3. Install joystick driver from ROS

```
$ sudo apt-get install ros-kinetic-rosserial
$ sudo apt-get install ros-kinetic-joy
```

4. Create a symbolic link between repository to the catkin space

```
ln -s /path/to/this/repo/ $HOME/catkin_ws/src
```


## Old log

Autonomous RC Race Car using ROS and Keras/TensorFlow. Inspired by JetsonHacks

<p align="center">
<img src="https://github.com/dat-ai/jetson-car/raw/master/docs/img/car.jpg" width="720">
</p>

Repository Contents
-------------------
* **`/docs`** - research papers, documents, photos related to this project
* **`/setup`** - Firmwares for Teensy 3.2 & SparkFun IMU Sensor + Set up instructions for Jetson TK1
* **`/src`** - ROS source directory (ROS Nodes and Launch files)


Setup Instruction:
------------------
Please check [`docs/setup/README.md`](./docs/setup/README.md)


Usage Example
-------------

* Open a new terminal
```shell
roscore
```

* Open another new terminal
```shell
roslaunch controller manual.launch
```

* Use joystick to test drive around
<p align="center">
<img src="https://github.com/dat-ai/jetson-car/raw/master/docs/img/controller.jpg" width="640">
</p>
* From Joystick, press Right Trigger button to activate Autopilot 

* In order to stop, press the small Right button to stop Autopilot


Author(s):
----------
* [**Dat Nguyen**](https://github.com/dat-ai)
* I am very happy if anyone would like to work with me on this project. please shoot me an email at tdat.nguyen93@gmail.com

Acknowledgements:
-----------------
* [JetsonHacks](http://www.jetsonhacks.com/) website provides helpful tips and tricks to work with Jetson TK1. JetsonHacks nicely simplifies the installation process with many ready-to-use scripts
* [DuckieTown](http://duckietown.mit.edu), an MIT Course,  is a great place to learn about developing autonomous vehicles.
* UPenn F1/10 Tutorials give a nice introduction to ROS, Hardware installation, and System Architecture for RC Car.
