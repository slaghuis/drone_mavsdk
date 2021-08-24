# drone_mavsdk
Base controller, exposing ROS2 interface to MAVSDK to control drone in local flight.  This node depends on the [drone_interfaces package](https://github.com/slaghuis/drone_interfaces).

## Coordinate Systems
PX4 uses a Foreward Right Down coordinate system, and ROS2 uses a Foreward Left Up coodinate system.  To align the two, a static transform boradcaster has to be started.  This is demonstrated in the launch file.

# node info
```
eric@simulator:~/ros_ws$ ros2 node info /drone_node
/drone_node
  Subscribers:
    /drone/cmd_vel: geometry_msgs/msg/Twist
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /drone/odom: nav_msgs/msg/Odometry
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /drone/arm: drone_interfaces/srv/Arm
    /drone/offboard: drone_interfaces/srv/Offboard
    /drone_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /drone_node/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /drone_node/get_parameters: rcl_interfaces/srv/GetParameters
    /drone_node/list_parameters: rcl_interfaces/srv/ListParameters
    /drone_node/set_parameters: rcl_interfaces/srv/SetParameters
    /drone_node/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:  
  
  Action Servers:
    /drone/land: drone_interfaces/action/Land
    /drone/takeoff: drone_interfaces/action/Takeoff
  Action Clients:
  
eric@simulator:~/ros_ws$
```
# Status
Test scripts run 100% in a simulator environment.  Code is stable.
Next Test : Compile on a Raspberry Pi, followed by a live flight test
## Test Scripts
Arm the drone
```
ros2 service call /drone/arm drone_interfaces/srv/Arm
```

Takeoff
```
ros2 action send_goal /drone/takeoff "drone_interfaces/action/Takeoff" "{target_altitude: 5.0}"
```

Land
```
ros2 action send_goal /drone/land "drone_interfaces/action/Land" "{gear_down: true}"
```

Enable offboard mode
```
ros2 service call /drone/offboard drone_interfaces/srv/Offboard "{enable: 0}"
```

Fly in a circle
```
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

# Caution
This code has not flown.
