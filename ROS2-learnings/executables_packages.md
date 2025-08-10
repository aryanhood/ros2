# ROS 2 Learning Notes (Humble)

This document contains quick reference commands, concepts, and examples for ROS 2 Humble.

---

## Installing ROS 2 Package

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```
## See list of packages

```bash
ros2 pkg list
```

## See list of <packages, executables>

```bash 
ros2 pkg executables
```

## See list of executables from turtlesim package

```bash
ros2 pkg executables turtlesim
```
## Where is turtlesim?

```bash
cd /opt/ros/humble
code /
Ctrl+P, turtlesim
```
## Should see `turtlesim` is located in

```bash
share/ament_index/resource_index/packages
```

## Run turtlesim. Do `ros2 run -h` so see options

```bash
ros2 run <package_name> <executable_name>
ros2 run turtlesim turtlesim_node
```
## In other ternimal, run the teleop node
```bash
ros2 run turtlesim turtle_teleop_key
```