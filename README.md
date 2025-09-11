# SchedUAV 
This is an end-to-end evaluation framework for UAVs developed by Sun Yat-sen University.

---

## Table of Contents

- [Features](#features)
- [Repository Structure](#repository-structure)
- [Environment & Platforms](#environment--platforms)
- [Quick Start](#quick-start)
- [Step-by-Step Installation](#step-by-step-installation)
  - [1) Get the Repository](#1-get-the-repository)
  - [2) Install System Dependencies (ROS/Drivers)](#2-install-system-dependencies-rosdrivers)
  - [3) Install & Build Livox-SDK2](#3-install--build-livox-sdk2)
  - [4) Build ROS Workspaces](#4-build-ros-workspaces)
  - [5) PX4 Setup & Build (SITL/HITL)](#5-px4-setup--build-sitlhitl)
- [Run Example](#run-example)
- [Environment Variables & Configuration](#environment-variables--configuration)
- [FAQ](#faq)
- [Roadmap](#roadmap)
- [License](#license)

---

## Features

- **End-to-end loop**: Fast-LIO + EGO-Planner + PX4 (SITL/HITL).
- **Dual-architecture**: x86_64 for development/simulation, ARM (RK3588S) for deployment.
- **Modular**: Independent ROS workspaces; optional Realsense / Mid360 plugins.
- **Reproducible**: Pinned dependencies and startup order; sample worlds and launch script templates provided.

---

## Repository Structure

```text
SCHEDUAV/
├─ PX4-Autopilot/          # PX4 source & builds (SITL/HITL)
├─ Livox-SDK2/             # Livox driver SDK
├─ ROS/                    # ROS1 workspaces
│  ├─ livox_ws/
│  ├─ fastlio_ws/
│  ├─ egoplanner_ws/
│  ├─ realsense_gazebo_ws/
│  └─ mid360_gazebo_ws/
├─ MyLaunch/               # Your launch files (e.g., MyGazebo.launch)
├─ MyModel/                # Custom models
├─ MyWorld/                # Custom worlds (e.g., MyForest.world)
├─ Scheduler/              # Scheduling-related code
├─ StressCPU/              # Load/interference scripts
├─ RunAll.bash             # One-click launch script (optional)
├─ README.md
└─ read.txt                # Notes for setup/run
```

---

## Environment & Platforms

- **OS**: Ubuntu 20.04 (ROS Noetic)
- **Simulator**: Gazebo Classic
- **Hardware**: x86_64 / RK3588S (A76/A55)

---

## Quick Start

```bash
# 1) Clone (replace with your remote if needed)
git clone https://github.com/author-name-omitted972/SchedUAV.git --recursive
cd SCHEDUAV

# 2) Common dependencies
sudo apt update
sudo apt install -y libpcl-dev libeigen3-dev libgoogle-glog-dev \
  libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev \
  libarmadillo-dev ros-noetic-ddynamic-reconfigure

# 3) Livox-SDK2
cd Livox-SDK2 && mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
cd ../../

# 4) Example ROS workspace
cd ROS/livox_ws
rosdep install --from-paths src --ignore-src -r
catkin_make
source devel/setup.sh
```

---

## Step-by-Step Installation

### 1) Get the Repository

```bash
git clone https://github.com/author-name-omitted972/PaperCode.git
cd SCHEDUAV
```

### 2) Install System Dependencies (ROS/Drivers)

```bash
sudo apt install -y build-essential cmake git python3-catkin-tools \
  libpcl-dev libeigen3-dev libgoogle-glog-dev libgflags-dev \
  libatlas-base-dev libsuitesparse-dev libceres-dev libarmadillo-dev \
  ros-noetic-ddynamic-reconfigure

sudo rosdep init || true
rosdep update
```

### 3) Install & Build Livox-SDK2

```bash
cd Livox-SDK2
mkdir -p build && cd build
cmake . && make -j$(nproc)
sudo make install
```

### 4) Build ROS Workspaces

```bash
# livox_ros_driver2
cd ROS/livox_ws && rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh

# Fast-LIO
cd ../fastlio_ws && rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh

# MAVROS + GeographicLib datasets
sudo apt install -y ros-noetic-mav*
cd /opt/ros/noetic/lib/mavros && sudo ./install_geographiclib_datasets.sh

# EGO-Planner
cd - && cd ../egoplanner_ws && rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh

# Realsense Gazebo plugin
cd ../realsense_gazebo_ws && rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh

# Mid360 simulation plugin
cd ../mid360_gazebo_ws && rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh
```

### 5) PX4 Setup & Build (SITL/HITL)

```bash
cd PX4-Autopilot
make clean distclean submodulesclean
bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Build SITL without launching
DONT_RUN=1 make px4_sitl_default gazebo-classic

# Gazebo plugin/model/library paths (adjust to your paths)
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(pwd)/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$(pwd)/../MyModel
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/build/px4_sitl_default/build_gazebo-classic
```

---

## Run Example

```bash
# 1) Gazebo world (replace with your actual world)
roslaunch MyLaunch/MyGazebo.launch world_name:=$(pwd)/MyWorld/MyForest.world

# 2) PX4 (example: HITL)
cd PX4-Autopilot
sudo ./bin/px4 -s HITL.config

# 3) ROS nodes
roslaunch fast_lio mapping_mid360.launch
roslaunch ego_planner single_run_in_exp.launch
roslaunch MyMAVROS.launch

# 4) Control/goal
rosrun ego_planner offboard_control
rosrun ego_planner pub_goal
```

---

## Environment Variables & Configuration

```bash
# Typical example (adjust paths to your setup)
export GAZEBO_PLUGIN_PATH=.:<PX4>/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=.:<PX4>/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$(pwd)/MyModel
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<PX4>/build/px4_sitl_default/build_gazebo-classic
```

---

## FAQ

```text
1) Plugin/model not found: verify that the three env vars contain PX4 build paths and your custom model path.
2) MAVROS GeographicLib error: run /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh.
3) RK3588S graphics: install libgl1-mesa-glx libgl1-mesa-dri libglx-mesa0 and reinstall Qt graphics libs if needed.
4) Large latency/jitter: fix CPU frequencies / pin cores, isolate background services, lower sim resolution/textures/lighting.
```

---

## Roadmap

```text
- One-click launcher scripts/launch_all.sh
- Dockerized dev environment
- More worlds and tasks (Forest / Pillars / City)
- Reproducible guides for typical scheduling configs (CFS / RM / EDF)
```

---

## License

```text
Apache-2.0
```

