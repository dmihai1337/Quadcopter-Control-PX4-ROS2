# Autonomous-Drone-Control 

## Task Description
The task is to implement autonomous drone control for a multicopter to perform a simple mission consisting of arming, taking off, visiting a set of waypoints, followed by RTL or landing, while handling basic flight states and safety conditions. This is done using ROS 2 and PX4 Autopilot with Gazebo for a SITL implementation. In the following I will go through my work and my thought process at every step.

## Environment
The first hours were spent setting up the technical environment. The starting point was an Ubuntu 22.04 installation on a Lenovo Thinkpad P1 Gen 5. 

`ROS2 Humble` was installed using the [official documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), followed by the installation of `Gazebo Harmonic`. It's worth mentioning that the officially supported Gazebo version for Humble is Gazebo Fortress, but PX4 is more compatible with Harmonic, so as a workaround, we can install the binaries for Gazebo Harmonic and the apt packages for ros_gz bridge separately, as detailed [here](https://gazebosim.org/docs/latest/ros_installation/#gazebo-harmonic-with-ros-2-humble-or-rolling-use-with-caution). 

The next step is setting up `PX4 Autopilot`, which is done by cloning the repo and building the code, as detailed in the [official documentation](https://docs.px4.io/main/en/dev_setup/getting_started.html). To keep a clean environment I have set up a python virtual environment with `venv`, which can be installed through the apt pacakge python3-venv. All dependencies related to this project are pip-installed there. There are two important aspects here regarding the given installation script. Firstly, we only need to run simulation, so we don't use real hardware. We can thus comment out the NuttX toolchain. Secondly, since we installed Gazebo separately as a personal preferance, we can also comment out the Gazebo installation from the installation script and install the rest.

In order to facilitate communication between PX4 Autopilot and ROS2 we need to set up a piece of middleware called `uXRCE-DDS` as detailed in the [PX4 documentation for ROS2](https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client), which allows the uORB messages of PX4 to be published and subscribed to as though they were ROS topics. An issue was encountered here while building from source, where, in the CMakeLists.txt a branch is specified that does not exist anymore. Referencing a newer, existing branch instead, fixed the problem.

In every ROS2 x PX4 workspace there needs to be a copy of the package [`px4_msgs`](https://github.com/PX4/px4_msgs), which contains ROS2 message definitions for PX4. The package [`px4_ros_com`](https://github.com/PX4/px4_ros_com) contains example nodes for exchanging data and commands between ROS2 and PX4, so we'll clone it as well to get some boilerplate from it.

The last tool that we need for now is the Ground Control Station [`QGroundControl`](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu), for communication with the drone.



