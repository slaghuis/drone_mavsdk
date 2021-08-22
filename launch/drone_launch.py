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
                {"connection_url": "udp://:14540"}
             ]
           )
    tf = Node(package = 'tf2_ros',
                       executable = 'static_transform_publisher',
                       arguments = ['0', '0', '0',  '0' , '0', '3.14159', 'odom', 'base_link'])

    ld.add_action(node)
    ld.add_action(tf)

    return ld
