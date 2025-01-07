from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare 'streaming' launch argument
    streaming_arg = DeclareLaunchArgument(
        'streaming',
        default_value='false',
        description='Enable or disable streaming'
    )

    # Node definition for ROSA Turtle agent
    turtle_agent_node = Node(
        package='turtle_agent',               # ROS 2 package name
        executable='turtle_agent.py',         # Node executable (Python script)
        name='rosa_turtle_agent',             # Node name
        parameters=[{'streaming': LaunchConfiguration('streaming')}],
        output='screen',                      # Print logs to screen
        respawn=False                         # No respawn on failure
    )
    
    # Node definition for Turtlesim 
    turtlesim_node = Node(
        package='turtlesim',        # Package name
        executable='turtlesim_node', # Executable for turtlesim
        name='turtlesim',            # Name of the node
        output='screen'              # Log output to the screen
    )

    interactive_service_node = Node(
        package='turtle_agent',  # Replace with your package name
        executable='service_node.py',  # Replace with your node's executable name
        name='interactive_service',
        output='screen'
    )

    return LaunchDescription([
        streaming_arg,  # Add the streaming argument
        turtle_agent_node,  # Add the node to launch
        turtlesim_node,
        interactive_service_node
    ])
