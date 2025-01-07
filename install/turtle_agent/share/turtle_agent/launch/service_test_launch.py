from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    interactive_service_node = Node(
        package='turtle_agent',  # Replace with your package name
        executable='service_node.py',  # Replace with your node's executable name
        name='interactive_service',
        output='screen'
    )

    return LaunchDescription([
        interactive_service_node
    ])