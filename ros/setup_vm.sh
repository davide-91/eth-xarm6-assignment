#!/bin/bash
# Sets up the RViz visualization of the cell on Ubuntu 20.04 + ROS Noetic.
# Usage (after cloning this repository to ~/eth_repo):  bash ~/eth_repo/ros/setup_vm.sh
set -e
source /opt/ros/noetic/setup.bash

echo "== 1/4 Installing the ROS tools used by the launch file"
sudo apt install -y git ros-noetic-xacro ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher ros-noetic-rviz

echo "== 2/4 Downloading the official UFACTORY robot model (xarm_description)"
mkdir -p ~/xarm_ws/src
if [ ! -d ~/xarm_ros_full ]; then
  git clone --depth 1 https://github.com/xArm-Developer/xarm_ros.git ~/xarm_ros_full
fi
rm -rf ~/xarm_ws/src/xarm_description ~/xarm_ws/src/xarm_cell_viz
cp -r ~/xarm_ros_full/xarm_description ~/xarm_ws/src/

echo "== 3/4 Copying the cell visualization package"
cp -r "$(dirname "$0")/xarm_cell_viz" ~/xarm_ws/src/
chmod +x ~/xarm_ws/src/xarm_cell_viz/scripts/cell_poses.py

echo "== 4/4 Building the workspace"
cd ~/xarm_ws && catkin_make
grep -q "xarm_ws/devel/setup.bash" ~/.bashrc || echo "source ~/xarm_ws/devel/setup.bash" >> ~/.bashrc

echo
echo "Done. Open a NEW terminal and run:"
echo "  roslaunch xarm_cell_viz cell.launch scene:=r1_place_far"
