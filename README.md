# SchedUAV 
```text
This is an end-to-end simulation framework for UAVs.
```


## Project Layout

```text
SCHEDUAV/
├─ PX4-Autopilot/          # PX4 source & builds
├─ Livox-SDK2/             # Livox driver SDK
├─ ROS/
│  ├─ livox_ws/            # livox_ros_driver2
│  ├─ fastlio_ws/          # Fast-LIO
│  ├─ egoplanner_ws/       # EGO-Planner
│  ├─ realsense_gazebo_ws/ # Realsense Gazebo plugin (Host)
│  └─ mid360_gazebo_ws/    # Mid360 simulation plugin (Host)
├─ Launch/                 # Gazebo launch (e.g., Gazebo.launch)
├─ Model/                  # Custom models
├─ World/                  # Custom worlds (e.g., Forest.world)
├─ Scheduler/              # Custom scheduler (e.g., CFS, RM, EDF)
├─ StressCPU/              # CPU interference
└─ README.md
```

---

## Common Setup (execute on both Host and Board)


```bash
#Clone the repository
git clone https://github.com/author-name-omitted972/SchedUAV.git --recursive
cd SchedUAV
git submodule update --init --recursive

# Install ROS Noetic
wget http://fishros.com/install -O fishros && . fishros

# ROS dependencies
sudo apt update
sudo apt install -y build-essential cmake git python3-catkin-tools \
  libpcl-dev libeigen3-dev libgoogle-glog-dev libgflags-dev \
  libatlas-base-dev libsuitesparse-dev libceres-dev libarmadillo-dev \
  ros-noetic-desktop-full ros-noetic-ddynamic-reconfigure

# MAVROS + GeographicLib
sudo apt install -y ros-noetic-mav*
cd /opt/ros/noetic/lib/mavros && sudo ./install_geographiclib_datasets.sh

# rosdep (first time)
sudo rosdep init || true
rosdep update
```

(Optional) RK3588S graphics runtime fix:

```bash
sudo apt install -y libgl1-mesa-glx libgl1-mesa-dri libglx-mesa0
sed -i 's/.*wiki.t-firefly.com.*/\#&/' /etc/apt/sources.list
sudo apt reinstall -y libqt5gui5 libqt5opengl5-dev
sed -i '/.*wiki.t-firefly.com.*/s/^#//' /etc/apt/sources.list
```

---

## Host Workflow (sim packages + PX4 cross-compile & upload)



### 1) Build Gazebo-related ROS packages

```bash
# Realsense Gazebo plugin
cd ROS/realsense_gazebo_ws
rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh

# Mid360 simulation plugin
cd ../mid360_gazebo_ws
rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh
```

### 2) Cross-compile PX4 and upload to Board via SSH (HITL)

```bash
cd PX4-Autopilot
make clean distclean submodulesclean
bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

export AUTOPILOT_HOST=<board-host-or-ip>
export AUTOPILOT_USER=<ssh-username>

make firefly_rk3588s_arm64
make firefly_rk3588s_arm64 upload
```

> Ensure `ssh ${AUTOPILOT_USER}@${AUTOPILOT_HOST}` works (keys or password).

---

## Board Workflow (sensors & algorithm stack)

### 1) Livox-SDK2 and livox_ros_driver2

```bash
# Livox-SDK2
cd Livox-SDK2 && mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install

# livox_ros_driver2
cd ../../ROS/livox_ws
rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh
```

### 2) Fast-LIO and EGO-Planner

```bash
cd ../fastlio_ws && rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh

cd ../egoplanner_ws && rosdep install --from-paths src --ignore-src -r
catkin_make && source devel/setup.sh
```

### 3) Scheduler and Stressor

```bash
cd Scheduler/build
cmake .. && make

cd StressCPU/build
cmake .. && make
```

---

## Run (HITL: Host simulation + Board PX4)

**Network**

```bash
# Host(~/.bashrc or Every Terminal)
export ROS_IP=HOST_IP
roscore
```

```bash
# Board(~/.bashrc or Every Terminal)
export ROS_MASTER_URI=http://HOST_IP:11311/
export ROS_IP=BOARD_IP
```

**Host**

```bash
# Gazebo world (folders are Launch/Model/World)
REPO_ROOT=$(pwd)
roslaunch Launch/Gazebo.launch world_name:=${REPO_ROOT}/World/Forest.world
```

**Board**

```bash
# System Setup
sudo cset shield --cpu 4-7 --kthread on
echo 0 | sudo tee /sys/fs/cgroup/cpuset/cpuset.sched_load_balance
echo -1 | sudo tee /proc/sys/kernel/sched_rt_runtime_us

# Board-side task chain
roslaunch fast_lio mapping_mid360.launch

roslaunch ego_planner single_run_in_exp.launch

rosrun ego_planner offboard_control

rosrun ego_planner pub_goal

cd ~/px4 
sudo ./bin/px4 -s HITL.config  # Start the  PX4

roslaunch Launch/Mavros.launch

# PX4
commander mode offboard # Switch Mode 
commander arm # Arm the drone and takeoff

# Scheduler
cd Scheduler/build
sudo ./cfs
sudo ./rms
sudo ./edf

# StressCPU
cd StressCPU/build
sudo cset shield --exec -- ./StressCPU --cpu 4,5,6,7 --no-stagger --period 1 0.1 4

# Trace
sudo trace-cmd record -b 65536 -M F0 -e sched_wakeup -e sched_switch -e sys_enter_kill
sudo trace-cmd report -F 'sys_enter_kill' -F 'sched_wakeup' -F 'sched_switch' -i trace.dat > trace.txt
```

---

## Environment Variables & Configuration (Host example)

```bash
# Use repo root as base
export REPO_ROOT=$(pwd)

# Gazebo
export GAZEBO_PLUGIN_PATH=${REPO_ROOT}/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=${REPO_ROOT}/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${REPO_ROOT}/Model
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${REPO_ROOT}/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic

# PX4 upload
export AUTOPILOT_HOST=<board-host-or-ip>
export AUTOPILOT_USER=<ssh-username>
```

---

## FAQ

- Plugin/model not found: verify `GAZEBO_PLUGIN_PATH / GAZEBO_MODEL_PATH / LD_LIBRARY_PATH` point to PX4 build and Model paths.  
- MAVROS GeographicLib error: run `/opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh` on board.  

---

## License

Apache-2.0













