1. Native Installation

Note that ROS only works on Linux. You will need to install Ubuntu 16.04 if you want a native installation, or use the provided Docker file to run a preconfigured ROS installation on Ubuntu 16.04 inside a virtual machine on your Mac or Windows host.

### Issue 1: catkin_make error: "Could not find a package configuration file provided by "pcl_ros

Can be solved with:
```
sudo apt install ros-kinetic-pcl-ros
```
[Here's a reference of this issue](https://github.com/udacity/CarND-Capstone/issues/125)

### Issue 2: Cannot not found command catkin_make, roscore, rosbag and etc.

Every time we open a terminal, we need to type `source /opt/ros/kinetic/setup.bash` in command line if we do not setup ROS environment variables when installing ROS Kinetic. To set up the environment variables, use
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

2. Using Virtual Machine

### Issue 1: received `socket.timeout: timed out` error even after setting port forwarding.

Need to check whether port 4567 is in use. If anything in use, stop it and restart virtual machine.

```
lsof -i | grep 4567
```
[Here's a reference of this issue](https://discussions.udacity.com/t/ros-and-simulator-communication-socket-error/381345)

3. Docker Installation

### Issue 1: received  `cannot connect to X server` error when running rviz in Docker container.

Can be potentially solved using instructions [here](
http://wiki.ros.org/docker/Tutorials/GUI). But we suggest to NOT use rviz in docker container.
