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
   
    ld.add_action(node)

    return ld
