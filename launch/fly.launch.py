from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','map','odom']          
    )

    map_odom_ned_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','1.57', '0', '3.14','map','odom_ned']          
    )
    
    drone_node = Node(
             package="drone",
             executable="drone_node",
             name="drone_node",
             output="screen",
             emulate_tty=True,
             parameters=[
                {"connection_url": "serial:///dev/ttyAMA1:921600"},
                {"height_topic": "vl53l1x/range"},
                {"height_sensor_z_offset": 0.153},
                {"use_height_sensor": False}
             ]
    )
    
    odom_tf2_broadcaster = Node(
        package="drone",
        executable="odom_tf2_broadcaster",
        output="screen",
        emulate_tty=True
    )    


    ld.add_action(map_odom_tf)
    ld.add_action(map_odom_ned_tf)
    ld.add_action(drone_node)
    ld.add_action(odom_tf2_broadcaster)

    return ld
