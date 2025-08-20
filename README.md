# Create3 SLAM Package (ROS JAZZY)

This repository provides a working ROS 2 Jazzy package for running SLAM mapping with the iRobot Create3 and RPLIDAR A1.  

The official [create3_lidar_slam example](https://github.com/iRobotEducation/create3_examples/tree/jazzy/create3_lidar_slam) does not work correctly on Jazzy, so this repo contains a fixed and simplified version.

## Parts List

- Raspberry Pi&reg; 5 [¹](#trademark-attribution)
- USB-C&reg; to USB-C&reg; cable [²](#trademark-attribution)
- Slamtec RPLidar A1M8
- USB Micro-B to USB-A cable
- Some screws to fix these

## Setup

### Hardware Setup

The files in this example assume the **RPLIDAR A1** is mounted **12 mm behind the center of rotation** on the top of the Create 3 robot, in the arrangement shown below.  

The SLAM solver relies on a proper **tf tree**; if you mount the sensor in another location, you will need to modify the parameters in the **static transform publisher** launched from:`launch/sllidar_a1_launch_modified.py`

> ℹ️ This is a modified version of the file provided in the [Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) repository.

---

All STL files for brackets and mounts referenced in this example can be found in the official iRobot documentation here:  
[Create® 3 Compute and Mounting Hardware STLs](https://iroboteducation.github.io/create3_docs/hw/print_compute/)

### On the Raspberry Pi

If your RPLIDAR is attached to your Raspberry Pi⁵, you can connect to it via SSH:

```bash
ssh <username>@<ip-address>
```
Once you are connected to your Raspberry Pi, clone this repository, build it, and **source the setup script in every terminal you open**:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/SugandhMittal/Create3_Jazzy.git
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/local_setup.sh
```
Run the sensors launch script, which includes the LIDAR driver and transform from the laser scan to the robot (on the Raspberry Pi):

```bash
source ~/ros2_ws/install/local_setup.sh
ros2 launch Create3_jazzy sllidar_a1_launch_modified.py
```

### On the Computer
clone this repository, build it, and **source the setup script in every terminal you open**:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/SugandhMittal/Create3_Jazzy.git
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/local_setup.sh
```
Install Slam Toolbox if not already installed:
```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
```
If installation via apt fails, build from source:

```bash
cd ~/ros2_ws/src
git clone --branch jazzy https://github.com/SteveMacenski/slam_toolbox.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
After installing, start the SLAM Toolbox:
```bash
ros2 launch slam_toolbox online_async_launch.py 
```
You can switch `online_async_launch.py` with `online_sync_launch.py`, `offline_async_launch.py`, or `offline_sync_launch.py` depending on your use case.

In another terminal, drive the robot around:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Open another terminal, source the setup script again, and run RViz:
```bash
ros2 launch Create3_jazzy rviz_launch.py
```

> ℹ️ The RViz launch file is taken directly from the official [create3_lidar_slam example](https://github.com/iRobotEducation/create3_examples/tree/jazzy/create3_lidar_slam) repository.





---

## Trademark Attribution

¹ Raspberry Pi&reg; is a trademark of Raspberry Pi Trading.  
² USB-C&reg; is a trademark of USB Implementers Forum.  
