# Drone MAVSDK
Base controller, exposing a ROS2 interface to MAVSDK to control drone in local flight.  This node depends on the [drone_interfaces package](https://github.com/slaghuis/drone_interfaces).

## Coordinate Systems
PX4 uses a NED coordinate system, and ROS2 uses a ENU coodinate system.  A transform broadcaster is included to publish a suitible transform from odom->base_link.  The drone node published odomety in the DED frame.  A second node reads this transform and published the odom and base_link transforms, conpleting the tree in line with [ROS-REP 105](https://ros.org/reps/rep-0105.html).  All unit and coordinate conventions in this node are compliant to [ROS-REP 103](https://www.ros.org/reps/rep-0103.html).

# node info
```
xxx@simulator:~/ros_ws$ ros2 node info /drone_node
/drone_node
  Subscribers:
    /drone/cmd_vel: geometry_msgs/msg/Twist
    /vl53l1x/range: sensor_msgs/msg/Range
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
Test scripts run 100% in a simulator environment.  Code is stable.
Compiles on a Raspberry Pi 3 and 4, and connects successfully to a Pixhawk 4 Mini.

Next Test : A live flight test

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
