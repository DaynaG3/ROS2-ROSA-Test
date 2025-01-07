#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from math import cos, sin, sqrt
from typing import List

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, TeleportRelative, SetPen
from turtlesim.msg import Pose
from langchain.agents import tool

from typing import Annotated #Added
#from langchain_core.tools import tool #Added

cmd_vel_pubs = {}

#   # Add default turtle publisher
#   self.add_cmd_vel_pub("turtle1", self.create_publisher(Twist, '/turtle1/cmd_vel', 10))

def add_cmd_vel_pub(name: str, publisher: Publisher):
    global cmd_vel_pubs
    cmd_vel_pubs[name] = publisher

def remove_cmd_vel_pub(name: str):
    global cmd_vel_pubs
    cmd_vel_pubs.pop(name, None)

def within_bounds(x: float, y: float) -> tuple:
    """
    Check if the given x, y coordinates are within the bounds of the turtlesim environment.

    :param x: The x-coordinate.
    :param y: The y-coordinate.
    """
    if 0 <= x <= 11 and 0 <= y <= 11:
        return True, "Coordinates are within bounds."
    else:
        return False, f"({x}, {y}) is out of bounds. Range is [0, 11] for both axes."

def will_be_within_bounds(
    node: Node, name: str, velocity: float, lateral: float, angle: float, duration: float = 1.0
) -> tuple:
    """Check if the turtle will be within bounds after publishing a twist command."""
    pose = get_turtle_pose(node, [name])
    current_x = pose[name].x
    current_y = pose[name].y
    current_theta = pose[name].theta

    if abs(angle) < 1e-6:
        new_x = (
            current_x
            + (velocity * cos(current_theta) - lateral * sin(current_theta)) * duration
        )
        new_y = (
            current_y
            + (velocity * sin(current_theta) + lateral * cos(current_theta)) * duration
        )
    else: # Circular motion
        radius = sqrt(velocity**2 + lateral**2) / abs(angle)
        center_x = current_x - radius * sin(current_theta)
        center_y = current_y + radius * cos(current_theta)
        angle_traveled = angle * duration
        new_x = center_x + radius * sin(current_theta + angle_traveled)
        new_y = center_y - radius * cos(current_theta + angle_traveled)

        # Check if any point on the circle is out of bounds
        for t in range(int(duration) + 1):
            angle_t = current_theta + angle * t
            x_t = center_x + radius * sin(angle_t)
            y_t = center_y - radius * cos(angle_t)
            in_bounds, _ = within_bounds(x_t, y_t)
            if not in_bounds:
                return (
                    False,
                    f"The circular path will go out of bounds at ({x_t:.2f}, {y_t:.2f}).",
                )
    # Check if the final x, y coordinates are within bounds
    in_bounds, message = within_bounds(new_x, new_y)
    if not in_bounds:
        return (
            False,
            f"This command will move the turtle out of bounds to ({new_x:.2f}, {new_y:.2f}).",
        )

    return True, f"The turtle will remain within bounds at ({new_x:.2f}, {new_y:.2f})."

# Tool: Spawn a turtle
@tool
def spawn_turtle(node: Node, name: str, x: float, y: float, theta: float) -> str:

    """
    Spawn a turtle at the given x, y, and theta coordinates.

    :param name: name of the turtle.
    :param x: x-coordinate.
    :param y: y-coordinate.
    :param theta: angle.
    """

    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message
    
    name = name.replace("/", "")

    try:
        spawn = node.create_client(Spawn, '/spawn')
        spawn.wait_for_service(timeout_sec=5)
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        spawn.call_async(request)

        global cmd_vel_pubs
        cmd_vel_pubs[name] = node.create_publisher(Twist, f'/{name}/cmd_vel', 10)

        return f"{name} spawned at x: {x}, y: {y}, theta: {theta}."
    except Exception as e:
        return f"Failed to spawn {name}: {e}"

# Tool: Kill a turtle
@tool
def kill_turtle(node: Node, names: List[str]) -> str:

    """
    Removes a turtle from the turtlesim environment.

    :param names: List of names of the turtles to remove (do not include the forward slash).
    """
    names = [name.replace("/", "") for name in names]
    response = ""
    global cmd_vel_pubs

    for name in names:
        try:
            kill = node.create_client(Kill, f'/{name}/kill')
            kill.wait_for_service(timeout_sec=5)
            kill.call_async(Kill.Request())

            cmd_vel_pubs.pop(name, None)

            response += f"Successfully killed {name}.\n"
        except Exception as e:
            response += f"Failed to kill {name}: {e}\n"

    return response
    #     kill_client = node.create_client(Kill, f'/{name}/kill')
    #     if not kill_client.wait_for_service(timeout_sec=5.0):
    #         response += f"Service /{name}/kill unavailable.\n"
    #         continue

    #     request = Kill.Request()
    #     request.name = name

    #     future = kill_client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() is not None:
    #         node.remove_cmd_vel_pub(name)
    #         response += f"Successfully killed {name}.\n"
    #     else:
    #         response += f"Failed to kill {name}.\n"

    # return response

@tool
def clear_turtlesim(node: Node):
    """Clears the turtlesim background and sets the color to the value of the background parameters."""
    try:
        clear = node.create_client(Empty, '/clear')
        clear.wait_for_service(timeout_sec=5)
        clear.call_async(Empty.Request())
        return "Successfully cleared the turtlesim background."
    except Exception as e:
        return f"Failed to clear the turtlesim background: {e}"

@tool   
def teleport_absolute(
    node: Node, name: str, x: float, y: float, theta: float, hide_pen: bool = True
):
    """
    Teleport a turtle to the given x, y, and theta coordinates.

    :param name: name of the turtle
    :param x: The x-coordinate, range: [0, 11]
    :param y: The y-coordinate, range: [0, 11]
    :param theta: angle
    :param hide_pen: True to hide the pen (do not show movement trace on screen), False to show the pen
    """
    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message

    try:
        teleport = node.create_client(TeleportAbsolute, f'/{name}/teleport_absolute')
        teleport.wait_for_service(timeout_sec=5)

        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = x
        teleport_req.y = y
        teleport_req.theta = theta
        teleport.call_async(teleport_req)

        return f"{name} new pose: ({x}, {y}) at {theta} radians."
    except Exception as e:
        return f"Failed to teleport the turtle: {e}"

@tool   
def teleport_relative(node: Node, name: str, linear: float, angular: float):
        """
        Teleport a turtle relative to its current position.

        :param name: name of the turtle
        :param linear: linear distance
        :param angular: angular distance
        """
        client = node.create_client(TeleportRelative, f'/{name}/teleport_relative')
        
        # Wait for the service to be available
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f"Service /{name}/teleport_relative not available.")
            return f"Failed to teleport the {name}: service not available."
        
        # Create the request
        request = TeleportRelative.Request()
        request.linear = linear
        request.angular = angular


# Tool: Get pose of turtles
@tool
def get_turtle_pose(node: Node, names: List[str]) -> dict:
    """
    Get the pose of one or more turtles.

    :param names: List of names of the turtles to get the pose of.
    """
    names = [name.replace("/", "") for name in names]
    poses = {}
    for name in names:
        try:
            pose_msg = node.create_subscription(Pose, f'/{name}/pose', lambda msg: poses.update({name: msg}), 10)
            rclpy.spin_once(node)
            poses[name] = pose_msg
        except Exception as e:
            poses[name] = f"Error retrieving pose: {e}"
    return poses

# # Tool: Publish twist commands
# @tool
# def publish_twist_to_cmd_vel(node: Node, name: str, linear: float, angular: float, steps: int = 1):
#     """
#     Publish a Twist message to the /{name}/cmd_vel topic to move a turtle robot.
#     Use a combination of linear and angular velocities to move the turtle in the desired direction.

#     :param name: name of the turtle (do not include the forward slash)
#     :param velocity: linear velocity, where positive is forward and negative is backward
#     :param lateral: lateral velocity, where positive is left and negative is right
#     :param angle: angular velocity, where positive is counterclockwise and negative is clockwise
#     :param steps: Number of times to publish the twist message
#     """
#     name = name.replace("/", "")
#     if name not in node.cmd_vel_pubs:
#         return f"No publisher available for {name}."

#     publisher = node.cmd_vel_pubs[name]
#     twist = Twist()
#     twist.linear.x = linear
#     twist.angular.z = angular

#     for _ in range(steps):
#         publisher.publish(twist)
#         node.get_logger().info(f"Published {twist} to {name}/cmd_vel")
#         node.get_clock

@tool
def publish_twist_to_cmd_vel(
    node: Annotated[Node, "A ROS 2 Node instance used for publishing"],
    topic: Annotated[str, "The topic to publish the velocity commands (e.g., /cmd_vel)"],
    linear_x: Annotated[float, "Linear velocity in the x-axis"],
    linear_y: Annotated[float, "Linear velocity in the y-axis (optional)"] = 0.0,
    angular_z: Annotated[float, "Angular velocity around the z-axis"] = 0.0,
) -> str:
    """
    Publishes velocity commands to a specified cmd_vel topic.

    :param node: A ROS 2 Node instance.
    :param topic: The topic to which the velocity command is published.
    :param linear_x: Linear velocity in the x direction.
    :param linear_y: Linear velocity in the y direction (optional).
    :param angular_z: Angular velocity around the z-axis.
    :return: Status message about the publishing action.
    """
    try:
        # Create publisher
        cmd_vel_pub = node.create_publisher(Twist, topic, 10)
        
        # Create and populate Twist message
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = angular_z

        # Publish message
        cmd_vel_pub.publish(cmd)

        return f"Published velocity command to {topic}: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}"
    except Exception as e:
        return f"Failed to publish velocity command: {e}"


#Tool: Stop turtle
@tool
def stop_turtle(node: Node, name: str):
    """
    Stop a turtle by publishing a Twist message with zero linear and angular velocities.

    :param name: name of the turtle
    """
    return publish_twist_to_cmd_vel(
        node,
        name,
        0.0,
        0.0,
        0.0,
    )

#Tool: Reset turtlesim
@tool
def reset_turtlesim(node: Node):
    """
    Resets the turtlesim, removes all turtles, clears any markings, and creates a new default turtle at the center.
    """
    try:
        reset = node.create_client(Empty, '/reset')
        reset.wait_for_service(timeout_sec=5)
        reset.call_async(Empty.Request())
        cmd_vel_pubs.clear()
        cmd_vel_pubs["turtle1"] = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        return "Successfully reset the turtlesim environment."
    except Exception as e:
        return f"Failed to reset the turtlesim environment: {e}"

#Tool: set pen colour
@tool
def set_pen(node: Node, name: str, r: int, g: int, b: int, width: int, off: int):
    """"
    Sets the pen properties for a turtle in the turtlesim environment.

    :param node: The ROS 2 node instance.
    :param name: The name of the turtle.
    :param r: Red color component (0-255).
    :param g: Green color component (0-255).
    :param b: Blue color component (0-255).
    :param width: Pen width.
    :param off: Whether the pen is off (1 for off, 0 for on).
    """
    name = name.replace("/", "")
    try:
        set_pen = node.create_client(SetPen, f'/{name}/set_pen')
        set_pen.wait_for_service(timeout_sec=5)
        set_pen_req = SetPen.Request()
        set_pen_req.r = r
        set_pen_req.g = g
        set_pen_req.b = b
        set_pen_req.width = width
        set_pen_req.off = off
        set_pen.call_async(set_pen_req)
        return f"Successfully set the pen color for {name}."
    except Exception as e:
        return f"Failed to set the pen color for {name}: {e}"