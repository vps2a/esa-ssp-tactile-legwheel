# ESA SSP Tactile LegWheel Project

ROS 2 workspace for leg-wheel sensing, RGB-D data collection, control and experiment logging for the ESA-SSP Legwheel project.

Filip Szczebak 2026

# Project Setup

## SSH Setup

### Setting up the ssh Access
1. Download the Remote-SSH extension and install it
2. In VS Code press Cmd + Shift + P
3. Search for the host you want or add one using ssh parallels@[IP_address] -> see section below on how to find the IP address
4. Then in the top bar again type > Remote-SSH Connect to Host... and find the IP Address you want

### Just the terminal

On the Paralles terminal (with ssh already enabled) run
hostname -I
Copy the IP address given
in VS Code terminal type
ssh parallels@[IP_address]

### ROS2 Sourceing

Remember to source ROS2 on each new ssh session with:
source /opt/ros/jazzy/setup.bash