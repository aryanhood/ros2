# ROS Parameters

Parameters are values you can change inside a node

## List of Parameters

```bash
ros2 param list
```
## Get parameter value

```bash
ros2 param get <node_name> <parameter_name>
ros2 
```
## Set Parameter value

```bash
ros2 param set <node_name> <parameter_name> <value>
ros2 param set /turtlesim background_g 255
```
## View all param for a node

```bash
ros2 param dump <node_name>
ros2 param dump /turtlesim
```
## Store param in vaml file

```bash
ros2 param dump /turtlesim > turtlesim.yaml
```
## load param from yaml file

```bash
ros2 param load /turtlesim turtlesim.yaml
```

## Load param on startup
```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
ros2 run turtlesim turtlesim_node --ros-args --params-file turtleism.yaml
```