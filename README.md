# drone_mavsdk
Base controller, exposing ROS2 interface to MAVSDK to control drone in local flight.  This node depends on the drone_interfaces package.

# node info
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

# Status
Test scripts run 100% in a simulator environment.  Code is stable.
Next Test : Compile on a Raspberry Pi, followed by a live flight test

# Caution
This one has not flown.  (I fried the Raspberry Pi trying to get it to speak to the flight controller.)
