# Autonomous-Drone-Control 

## Task Description
The task is to implement autonomous drone control for a multicopter to perform a simple mission consisting of arming, taking off, visiting a set of waypoints in offboard mode, followed by RTL or landing, while handling basic flight states and safety conditions. This is done using ROS 2 and PX4 Autopilot with Gazebo for a SITL implementation. In the following I will go through my work and my thought process at every step.

## Environment
The first hours were spent setting up the technical environment. The starting point was an Ubuntu 22.04 installation on a Lenovo Thinkpad P1 Gen 5. 

`ROS2 Humble` was installed using the [official documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), followed by the installation of `Gazebo Harmonic`. It's worth mentioning that the officially supported Gazebo version for Humble is Gazebo Fortress, but PX4 is more compatible with Harmonic, so as a workaround, we can install the binaries for Gazebo Harmonic and the apt packages for ros_gz bridge separately, as detailed [here](https://gazebosim.org/docs/latest/ros_installation/#gazebo-harmonic-with-ros-2-humble-or-rolling-use-with-caution). 

The next step is setting up `PX4 Autopilot`, which is done by cloning the repo and building the code, as detailed in the [official documentation](https://docs.px4.io/main/en/dev_setup/getting_started.html). To keep a clean environment I have set up a python virtual environment with `venv`, which can be installed through the apt pacakge python3-venv. All dependencies related to this project are pip-installed there. There are two important aspects here regarding the given installation script. Firstly, we only need to run simulation, so we don't use real hardware. We can thus comment out the NuttX toolchain. Secondly, since we installed Gazebo separately as a personal preferance, we can also comment out the Gazebo installation from the installation script and install the rest.

In order to facilitate communication between PX4 Autopilot and ROS2 we need to set up a piece of middleware called `uXRCE-DDS` as detailed in the [PX4 documentation for ROS2](https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client), which allows the uORB messages of PX4 to be published and subscribed to as though they were ROS topics. An issue was encountered here while building from source, where, in the CMakeLists.txt a branch is specified that does not exist anymore. Referencing a newer, existing branch instead, fixed the problem.

In every ROS2 x PX4 workspace there needs to be a copy of the package [`px4_msgs`](https://github.com/PX4/px4_msgs), which contains ROS2 message definitions for PX4. The package [`px4_ros_com`](https://github.com/PX4/px4_ros_com) contains example nodes for exchanging data and commands between ROS2 and PX4, so we'll clone it as well to get some boilerplate from it.

The last tool that we need for now is the Ground Control Station [`QGroundControl`](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu), for communication with the drone.

## Journal

In the following I present the steps I took to get to the solution, in order to make my reasoning and my decisions along the way, transparent.

### Phase 1 - Simple FSM for take-off & landing

After setting up the environment, testing that everything runs and testing drone control with an Xbox Controller, the first step is to run a given example from px4_ros_com. Inside the package, the file `offboard_control_srv.cpp` in `src/examples/offboard/` implements a ROS2 node in C++ which controls the drone through an FSM, which is a good starting point. The example *arms* the vehicle, turns on *offboard control mode* and *takes off* to an altitude of 5 meters. My first experiment was to extend this FSM to also handle *landing*, after reaching altitude. For this, 2 subscriptions were added, namely the first for listening to the local vehicle position in order to get the z coordinate and check when altitude is reached, and the second to listen to the land detected flag, after landing, to log the landing event and shutdown the node, giving back control to the Ground Control Station. The node communicates with PX4 through the vehicle command service, so we also get feedback on the command state, and have a basis for error handling. In order for the subscriptions to work, the QoS profile had to be set to a depth of 1, KeepLast, TransientLocal & BestEffort. I uploaded the px4_ros_com package for now, which can be cloned into the ROS2 workspace next to the `px4_msgs` package, which is a dependency, as previously mentioned.

### Phase 2 - Extend FSM to complete a square
The next step was extending the FSM from the previous phase by adding an extra state for traveling to each corner of a square in offboard mode. After arming and taking off at coordinates `{0, 0, -5}`, the vehicle visits the coordinates `{10, 0, -5}` -> `{10, 10, -5}` -> `{0, 10, -5}` -> `{0, 0, -5}` in a NED coordinate frame, before landing and disarming, finishing the mission. The points are published on the `fmu/in/trajectory_setpoint` topic. The arrival at every corner is checked by computing the L2 distance between the current (x, y) position and the goal (x, y) coordinates against a tolerance `eps = 0.035` which was set experimentally. This version is also uploaded to the commit history and the source is the same file, `offboard_control_srv.cpp` in `src/examples/offboard/` of the `px4_ros_com` package.
