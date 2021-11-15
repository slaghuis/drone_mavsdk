from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    node = Node(
             package="drone",
             executable="drone_node",
             name="drone_node",
             output="screen",
             emulate_tty=True,
             parameters=[
                {"connection_url": "udp://:14540"},
                {"height_topic": "vl53l1x/range"},
                {"height_sensor_z_offset": 0.153}
             ]
           )
   
    ld.add_action(node)

    return ld
