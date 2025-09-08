#!/usr/bin/env bash
set -euo pipefail

cd
# --- ROS 2 Humble prerequisites and installation ---

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools

source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# --- PX4 Autopilot clone and SITL build ---

if [[ ! -d PX4-Autopilot ]]; then
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
fi

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# --- Micro XRCE-DDS Agent installation ---

cd
if [[ ! -d Micro-XRCE-DDS-Agent ]]; then
  git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi

cd Micro-XRCE-DDS-Agent
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/

# --- Python dependencies ---

pip install --user -U empy==3.3.4 pyros-genmsg setuptools
sudo apt install -y git libclang-dev python3-vcstool
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git

# --- Rust ---

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# --- ROS2 packages for px4_msgs and rclrs ---

cd
mkdir -p ~/ws_ros_px4/src/
cd ~/ws_ros_px4/src/
if [[ ! -d px4_msgs ]]; then
  git clone https://github.com/PX4/px4_msgs.git
fi
cd ..
if [[ ! -d src/ros2_rust ]]; then
  git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
fi
vcs import src < src/ros2_rust/ros2_rust_humble.repos

# --- PX4 Gazebo models ---

cd
if [[ ! -d PX4-gazebo-models ]]; then
  git clone https://github.com/PX4/PX4-gazebo-models.git
fi

# --- QGroundControl Station ---

sudo usermod -aG dialout "$(id -un)"
sudo systemctl mask --now ModemManager.service
sudo apt remove --purge modemmanager
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

mkdir -p ~/qgc
curl -fL "https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage" -o ~/qgc/QGroundControl-x86_64.AppImage
chmod +x ~/qgc/QGroundControl-x86_64.AppImage

