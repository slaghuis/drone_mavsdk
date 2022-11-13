# Drone MAVSDK
Base controller, exposing a ROS2 interface to the PX4 flight controller using the MAVSDK library to control a drone in local flight.  This node depends on the [drone_interfaces package](https://github.com/slaghuis/drone_interfaces).

## Coordinate Systems
### Drone node
PX4 uses a NED coordinate system, and ROS2 uses an ENU coodinate system. This node publishes odometry in the odom_ned frame, and publishes an odom_ned -> base_link_ned transform.  A static transform broadcaster is needed to publish a map->odom_ned transform, effectively flipping and rotating NED into ENU.  
### Frame Broadcaser
A second node is included that is timer driven.  It looks up a transform from base_link_ned to odom, and publishes a transform from odom to base_link. This node needs a map->odom static transform broadcaster.  

The static transform broadcasters are defined in the launch file. This completes the tree in line with [ROS-REP 105](https://ros.org/reps/rep-0105.html).  All unit and coordinate conventions in this node are compliant to [ROS-REP 103](https://www.ros.org/reps/rep-0103.html).

# Node Info
```
xxx@simulator:~/ros_ws$ ros2 node info /drone_node
/drone_node
  Subscribers:
    /drone/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /drone/battery: sensor_msgs/msg/BatteryState
    /drone/gps: sensor_msgs/msg/NavSatFix
    /drone/odom: nav_msgs/msg/Odometry
    /tf: tf2_msgs/msg/TFMessage
  Service Servers:
    /drone/arm: drone_interfaces/srv/Arm
    /drone/offboard: drone_interfaces/srv/Offboard
  Service Clients:  
  
  Action Servers:
    /drone/land: drone_interfaces/action/Land
    /drone/takeoff: drone_interfaces/action/Takeoff
  Action Clients:
  
```
# Status
Test scripts run 100% in a simulator environment.  The code is stable.
Compiles on a Raspberry Pi 3 and 4, and connects successfully to a Pixhawk 4 Mini.

First test flights on an actual drone using a Pixhawk 4.0 Mini were good.  The code seems flight ready. 

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
ros2 service call /drone/offboard drone_interfaces/srv/Offboard "{enable: 1}"
```

Fly in a circle
```
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```
