# ROS Services

Services are used to communicate vetween nodes using aclient-server model, where the server responds when the client make a request.

## List all services name / Types

```bash
ros2 service list -t

# See service type
ros2 service type <service_name>
ros2 service type /clear
```

## Find service with specific type

```bash
ros2 service find <service_type>
ros2 service find std_srvs/srv/Empty
```

## See interface

```bash
ros2 interface show <service_type>
ros2 interface show turtlesim/srv/Spawn
```
## Calling a service

```bash
ros2 service call <service_name> <service>

# Clear drawing
ros2 service call /clear std_srvs/srv/Empty

# Spawn turtle 🐢
ros2 service call /spawn turtlesim/srv/Spawn
ros2 service call /spawn turtlesim/srv/Spawn "{x:2, y:2, theta: 0.2, name:''}"
```
