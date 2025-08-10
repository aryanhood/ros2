# ROS Topic

Using topics, one or more publishers can connect to one or more subscribers.

## Move the turtle anywhere 

```bash
ros2 run turtlesim_teleop_key
```

## List of Topics

```bash
ros2 topic list
# see details
ros2 topic list -t
```

## Start rqt graph

```bash
rqt_graph
```

## See topic output (move with teleop command output) XYZ co-ordinates 

```bash
ros2 topic echo <topic_name>
ros2 topic echo /turtle1/cmd_vel
```

## See Interface definition 

```bash
ros2 interface show <type>
ros2 interface show geometry_msgs/msg/Twist
```
*Expected output expresses velocity in free space, broken into XYZ linear and angular velocities.

## Publish data to a topic ("--once" means publish once and exit)

```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y:0.0, z: 1.8}}"
```

## Publish data to a topic with rate 1hz

```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y:0.0, z: 1.8}}"
```
*The turtle starts to move in circles.

## Echo the pose topic

```bash
ros2 topic echo /turtle1/pose
```
Open RQT Graph and click 'Refresh'

## View topic frequency

```bash
ros2 topic hz <topic_name>
ros2 topic hz /turtle1/pose
```
*Shows std rate, min, max, std deviation

