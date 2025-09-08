# Autonomous-Drone-Control 

## Task Description
The task is to implement autonomous drone control for a multicopter to perform a simple mission consisting of arming, taking off, visiting a set of waypoints in offboard mode, followed by RTL or landing, while handling basic flight states and safety conditions. This is done using ROS 2 and PX4 Autopilot with Gazebo for a SITL implementation. In the following I will go through my work and my thought process at every step.

<img width="3157" height="1510" alt="Screenshot of Drone Mission" src="https://github.com/user-attachments/assets/15f998ae-2ce6-4631-87cc-c846975bc913" />

## Environment
The first hours were spent setting up the technical environment. The starting point was an Ubuntu 22.04 installation on a Lenovo Thinkpad P1 Gen 5. 

`ROS2 Humble` was installed using the [official documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), followed by the installation of `Gazebo Harmonic`. It's worth mentioning that the officially supported Gazebo version for Humble is Gazebo Fortress, but PX4 is more compatible with Harmonic, so as a workaround, we can install the binaries for Gazebo Harmonic and the apt packages for ros_gz bridge separately, as detailed [here](https://gazebosim.org/docs/latest/ros_installation/#gazebo-harmonic-with-ros-2-humble-or-rolling-use-with-caution). 

The next step is setting up `PX4 Autopilot`, which is done by cloning the repo and building the code, as detailed in the [official documentation](https://docs.px4.io/main/en/dev_setup/getting_started.html). To keep a clean environment I have set up a python virtual environment with `venv`, which can be installed through the apt pacakge python3-venv. All dependencies related to this project are pip-installed there. There are two important aspects here regarding the given installation script. Firstly, we only need to run simulation, so we don't use real hardware. We can thus comment out the NuttX toolchain. Secondly, since we installed Gazebo separately as a personal preferance, we can also comment out the Gazebo installation from the installation script and install the rest. For convenience, when launching a Gazebo world, we can use the [`PX4-gazebo-models`](https://github.com/PX4/PX4-gazebo-models) repo and launch the simulation with a single python script.

In order to facilitate communication between PX4 Autopilot and ROS2 we need to set up a piece of middleware called `uXRCE-DDS` as detailed in the [PX4 documentation for ROS2](https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client), which allows the uORB messages of PX4 to be published and subscribed to as though they were ROS topics. An issue was encountered here while building from source, where, in the CMakeLists.txt a branch is specified that does not exist anymore. Referencing a newer, existing branch instead, fixed the problem.

In every ROS2 x PX4 workspace there needs to be a copy of the package [`px4_msgs`](https://github.com/PX4/px4_msgs), which contains ROS2 message definitions for PX4. The package [`px4_ros_com`](https://github.com/PX4/px4_ros_com) contains example nodes for exchanging data and commands between ROS2 and PX4, so we'll clone it as well to get some boilerplate from it. Since we will eventually work in Rust, we need to install [`Rust`](https://www.rust-lang.org/tools/install) and the [`rclrs`](https://github.com/ros2-rust/ros2_rust/blob/main/README.md) bindings. A new package is created with `cargo new` and we also need the cloned px4_msgs package in the same workspace since it's a dependency. 

The last tool that we need for now is the Ground Control Station [`QGroundControl`](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu), for communication with the drone.

The script `setup.sh` is intended to automate the entire environment setup discussed above. It is more convenient than a Dockerfile since all that is needed is to glue all the bash setup commands from the above frameworks into a single shell script. The script was, however, not tested. It's expected not to work right away but it should only need some small tweaks. I decided to invest the majority of my time in the implementation, which is discussed below.

## Journal

In the following I present the steps I took to get to the solution, in order to make my reasoning and my decisions along the way, transparent. I also present difficulties I encountered.

### Phase 1 - Simple FSM for take-off & landing

After setting up the environment, testing that everything runs and testing drone control with an Xbox Controller, the first step is to run a given example from px4_ros_com. Inside the package, the file `offboard_control_srv.cpp` in `src/examples/offboard/` implements a ROS2 node in C++ which controls the drone through an FSM, which is a good starting point. The example *arms* the vehicle, turns on *offboard control mode* and *takes off* to an altitude of 5 meters. My first experiment was to extend this FSM to also handle *landing*, after reaching altitude. For this, 2 subscriptions were added, namely the first for listening to the local vehicle position in order to get the z coordinate and check when altitude is reached, and the second to listen to the land detected flag, after landing, to log the landing event and shutdown the node, giving back control to the Ground Control Station. The node communicates with PX4 through the vehicle command service, so we also get feedback on the command state, and have a basis for error handling. In order for the subscriptions to work, the QoS profile had to be set to a depth of 1, KeepLast, TransientLocal & BestEffort. I uploaded the px4_ros_com package for now, which can be cloned into the ROS2 workspace next to the `px4_msgs` package, which is a dependency, as previously mentioned.

### Phase 2 - Extend FSM to complete a square

The next step was extending the FSM from the previous phase by adding an extra state for traveling to each corner of a square in offboard mode. After arming and taking off at coordinates `{0, 0, -5}`, the vehicle visits the coordinates `{10, 0, -5}` -> `{10, 10, -5}` -> `{0, 10, -5}` -> `{0, 0, -5}` in a NED coordinate frame, before landing and disarming, finishing the mission. The points are published on the `fmu/in/trajectory_setpoint` topic. The arrival at every corner is checked by computing the L2 distance between the current (x, y) position and the goal (x, y) coordinates against a tolerance `eps = 0.035` which was set experimentally. This version is also uploaded to the commit history and the source is the same file, `offboard_control_srv.cpp` in `src/examples/offboard/` of the `px4_ros_com` package.

### Phase 3 - Switch to Rust / rclrs bindings

Never having seen Rust code before, let alone write it, I spent around a day on crash courses and [The Rust Programming Language Book](https://doc.rust-lang.org/book/). After going through concepts like Ownership, Borrowing, Lifetimes and Error Handling through Options, I studied the ROS2 bindings for Rust - [rclrs](https://docs.rs/rclrs/latest/rclrs/). The setup consisted of building the new package with `cargo new` and adding rclrs and px4_msgs as dependencies in the `Cargo.toml` file, as well as in the `package.xml`. Those two packages have to be built in the workspace as well. Building with `colcon` is possible thanks to `ament-cargo`. Another great convenience is the automatic `rosidl` interface generation of the px4_msgs for Rust.

Starting from a bare-bones node implementation example, I added the *subscriptions* that were used before, namely for the topics `/fmu/out/vehicle_local_position_v1` and `/fmu/out/vehicle_land_detected`, using the `Worker` structure, that provides a state subscriptions can write to. Then the *publishers* for `/fmu/in/offboard_control_mode` and `/fmu/in/trajectory_setpoint` were added. Figuring out how to make state variables that can be generally read and written to took longer than expected. Since in Rust data can only have `one owner` at a time, we need to implement `shared ownership with mutual exclusion`. As such, wrapping the states in an `Arc<Mutex<...>>` is the solution, and when reading or writing the data in a spot, we first lock the Mutex. After testing the subscribers and publishers, the async *client* for the VehicleCommand service was implemented, along with the handling of the response. After implementing the rest of the functions, mainly for sending specific commands for arming, disarming, landing, the service was tested as well through those commands. Every needed communication functionality with PX4 is now implemented. The last step is thus to replicate the FSM and control an entire mission. The result of this phase is in the commit "Migration to Rust package", in the source file `offboard_control_srv.rs`. 

### Phase 4 - Rust FSM implementation

A major change is the removal of `Worker` structures, because of the constraints of working with them. Reading the worker's state involves calling the run() method of the worker, and the accessed data only exists in the scope of that call. Furthermore, in this phase, the C++ FSM logic from before was translated to Rust in order to control an entire mission through the node `offboard_control_srv`. The functionality is the same as before. The drone is armed, it takes off, completes a square, lands, disarms and the node shuts down. The resulting node is in the crate as part of the commit "Rust FSM takeoff, square and land".

Some language-specific tweaks have made the job more difficult than expected. Potential issues are `deadlocks`, where multiple locks are placed on state variables by mistake. This is not straightforward to see all the time, since locks can be placed at different locations, yet they are still in the same scope. In order to get to a deadlock-free solution I made small individual changes to design choices, which are not worth mentioning on their own. Another potential problem is `moving` variables, leaving None value in the original one. The state of the FSM is set at every step for the FSM to work, because when reading it, the `take()` method moves the actual value into the local variable and then it vanishes, leading to a `Panic` when reading the state in the next iteration. Last but not least, a problem I encountered was that commands were being sent correctly in order to get to a waypoint in offboard mode, but the timestamp for the `/fmu/in/trajectory_setpoint` publisher was the same all the time. As a result, the drone not only did not travel to the waypoint, but it also disarmed.

### Phase 5 - FSM redesigned for user-defined mission
The FSM was initially designed to complete a square. In the commit "FSM redesigned for custom mission" it has been redesigned and the circuit-specific states (for each of the 4 corners) were collapsed into a single `circuit` state, where the whole circuit is being processed. The user can define the circuit in the function `generate_circuit()` where the waypoints need to be given to define a waypoint vector. This function is called in the beginning. Furthermore, a more elegant way to go about status queries has been implemented, where all that the circuit state needs to do is call the `proceed()` function, which checks for completeness of the circuit, by returning one of 3 statuses -> `Progress::EnRoute` (traveling to next waypoint), `Progress::ArrivedAndNext` (arrived at waypoint, setting off for the next) or `Progress::ArrivedAndFinish` (arrived at last waypoint, circuit complete).

Some last details were changed in the commit "Refinement", according to the specifics of the task description, such as arming *before* switching to offboard mode, holding for 3 seconds after takeoff, triggering landing in case of failure, and console + csv file logging. The node now logs to the `console` and to the csv file `mission_log.csv`, which is saved in the directory where the node is run from. Last but not least, the commit "Heading control" also implements `heading` control for the drone to be aligned with its course, allowing for a Figure 8 maneuver as seen in the demo video.

A first improvement idea would be appending a timestamp to the csv log so it's not overwritten every time. The project could be extended by letting the user draw the mission circuit freely by hand, then convert those lines to a waypoint list. Another idea would be introducing more complex circuits, where the drone uses perception to scan for obstacles and plan its course on the spot. It has been a pleasure to work on this project and to learn by doing, so I wish to thank you for the opportunity and hope to hear from the team soon.

## Usage

In different terminals:

```bash
./qgc/QGroundControl-x86_64.AppImage
```

```bash
python PX4-gazebo-models/simulation-gazebo --world=baylands
```

```bash
MicroXRCEAgent udp4 -p 8888
```

```bash
cd PX4-Autopilot/
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=baylands make px4_sitl gz_x500
```

```bash
cd ws_ros_px4/
colcon build
source install/setup.bash
ros2 run offboard_control_srv offboard_control_srv
```

