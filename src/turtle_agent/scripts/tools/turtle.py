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
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute, TeleportRelative, Kill, SetPen

from typing import Annotated #Added
from langchain_core.tools import tool #Added
import time #Added

cmd_vel_pubs = {}

def add_cmd_vel_pub(name: str, publisher: Publisher):
    global cmd_vel_pubs
    cmd_vel_pubs[name] = publisher

def remove_cmd_vel_pub(name: str):
    global cmd_vel_pubs
    cmd_vel_pubs.pop(name, None)

# Add the default turtle1 publisher on startup
def init_node(node: Node):
    # rclpy.init()
    # node = Node('turtle_agent')
    add_cmd_vel_pub("turtle1", node.create_publisher(Twist, '/turtle1/cmd_vel', 10))
    return node

def within_bounds(x: float, y: float) -> tuple:
    if 0 <= x <= 11 and 0 <= y <= 11:
        return True, "Coordinates are within bounds."
    else:
        return False, f"({x}, {y}) will be out of bounds. Range is [0, 11] for each."

def will_be_within_bounds(
    node: Node, name: str, velocity: float, lateral: float, angle: float, duration: float = 1.0
) -> tuple:
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
    else:
        radius = sqrt(velocity**2 + lateral**2) / abs(angle)
        center_x = current_x - radius * sin(current_theta)
        center_y = current_y + radius * cos(current_theta)
        angle_traveled = angle * duration
        new_x = center_x + radius * sin(current_theta + angle_traveled)
        new_y = center_y - radius * cos(current_theta + angle_traveled)

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

    in_bounds, message = within_bounds(new_x, new_y)
    if not in_bounds:
        return (
            False,
            f"This command will move the turtle out of bounds to ({new_x:.2f}, {new_y:.2f}).",
        )

    return True, f"The turtle will remain within bounds at ({new_x:.2f}, {new_y:.2f})."

def spawn_turtle(node: Node, name: str, x: float, y: float, theta: float) -> str:
    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message

    name = name.replace("/", "")

    try:
        spawn = node.create_client(Spawn, '/spawn')
        spawn.wait_for_service(timeout_sec=5)
        spawn_req = Spawn.Request()
        spawn_req.x = x
        spawn_req.y = y
        spawn_req.theta = theta
        spawn_req.name = name
        spawn.call_async(spawn_req)

        global cmd_vel_pubs
        cmd_vel_pubs[name] = node.create_publisher(Twist, f'/{name}/cmd_vel', 10)

        return f"{name} spawned at x: {x}, y: {y}, theta: {theta}."
    except Exception as e:
        return f"Failed to spawn {name}: {e}"

def kill_turtle(node: Node, names: List[str]):
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
        except rclpy.exceptions.ROSInterruptException as e:
            response += f"Failed to kill {name}: {e}\n"

    return response

def clear_turtlesim(node: Node):
    try:
        clear = node.create_client(Empty, '/clear')
        clear.wait_for_service(timeout_sec=5)
        clear.call_async(Empty.Request())
        return "Successfully cleared the turtlesim background."
    except Exception as e:
        return f"Failed to clear the turtlesim background: {e}"

def get_turtle_pose(node: Node, names: List[str]) -> dict:
    names = [name.replace("/", "") for name in names]
    poses = {}

    for name in names:
        try:
            msg = node.create_subscription(Pose, f'/{name}/pose', lambda msg: msg, 10)
            poses[name] = msg
        except rclpy.exceptions.ROSInterruptException:
            return {"Error": f"Failed to get pose for {name}: /{name}/pose not available."}

    return poses

def teleport_absolute(
    node: Node, name: str, x: float, y: float, theta: float, hide_pen: bool = True
):
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

def teleport_relative(self, name: str, linear: float, angular: float):
        """
        Teleport a turtle relative to its current position.

        :param name: name of the turtle
        :param linear: linear distance
        :param angular: angular distance
        """
        client = self.create_client(TeleportRelative, f'/{name}/teleport_relative')
        
        # Wait for the service to be available
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service /{name}/teleport_relative not available.")
            return f"Failed to teleport the {name}: service not available."
        
        # Create the request
        request = TeleportRelative.Request()
        request.linear = linear
        request.angular = angular

#edited 10 Jan
@tool
def publish_twist_to_cmd_vel(
    name: Annotated[str, "The topic to publish the velocity commands (e.g., /cmd_vel)"],
    linear_x: Annotated[float, "Linear velocity in the x-axis"],
    linear_y: Annotated[float, "Linear velocity in the y-axis (optional)"] = 0.0,
    angular_z: Annotated[float, "Angular velocity around the z-axis"] = 0.0,
    steps: int = 1 ) -> str:
    """
    Publish a Twist message to the /{name}/cmd_vel topic to move a turtle robot.
    Use a combination of linear and angular velocities to move the turtle in the desired direction.

    :param name: name of the turtle (do not include the forward slash)
    :param linear_x: Linear velocity in the x direction, where positive is forward and negative is backward
    :param linear_y: Linear velocity in the y direction (optional), where positive is left and negative is right
    :param angular_z: Angular velocity around the z-axis, where positive is counterclockwise and negative is clockwise
    :param steps: Number of times to publish the twist message
    :return: Status message about the publishing action.
    """

    cmd = Twist()
    cmd.linear.x = linear_x
    cmd.linear.y = linear_y
    cmd.angular.z = angular_z 
    
    # Remove any forward slashes from the name
    name = name.replace("/", "")

    # Check if the movement will keep the turtle within bounds
    # in_bounds, message = will_be_within_bounds(
    # node, name, linear_x, linear_y, angular_z, duration=steps)

    try:

        # if name not in cmd_vel_pubs:
        #         return f"Publisher for {name} not found. Call create_cmd_vel_publisher() first."
        
        # # Create publisher
        # cmd_vel_pub = node.create_publisher(Twist, topic, 10)
    
        # if not in_bounds:
        #     return message     

        # Publish message
        global cmd_vel_pubs
        pub = cmd_vel_pubs[name]
        # cmd_vel_pubs[name].publish(cmd)
        for _ in range(steps):
            pub.publish(cmd)
            time.sleep(1.0)        
        return f"Published velocity command to {name}: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}"
    except Exception as e:
        return f"Failed to publish velocity command: {e}"

def stop_turtle(node: Node, name: str):
    return publish_twist_to_cmd_vel(
        node,
        name,
        0.0,
        0.0,
        0.0,
    )

def reset_turtlesim(node: Node):
    try:
        reset = node.create_client(Empty, '/reset')
        reset.wait_for_service(timeout_sec=5)
        reset.call_async(Empty.Request())
        cmd_vel_pubs.clear()
        cmd_vel_pubs["turtle1"] = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        return "Successfully reset the turtlesim environment."
    except Exception as e:
        return f"Failed to reset the turtlesim environment: {e}"

def set_pen(node: Node, name: str, r: int, g: int, b: int, width: int, off: int):
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
    
def has_moved_to_expected_coordinates(
    name: str, expected_x: float, expected_y: float, tolerance: float = 0.1
) -> str:
    """
    Check if the turtle has moved to the expected position.

    :param name: name of the turtle
    :param expected_x: expected x-coordinate
    :param expected_y: expected y-coordinate
    :param tolerance: tolerance level for the comparison
    """
    current_pose = get_turtle_pose.invoke({"names": [name]})
    current_x = current_pose[name].x
    current_y = current_pose[name].y

    distance = ((current_x - expected_x) ** 2 + (current_y - expected_y) ** 2) ** 0.5
    if distance <= tolerance:
        return (
            f"{name} has moved to the expected position ({expected_x}, {expected_y})."
        )
    else:
        return f"{name} has NOT moved to the expected position ({expected_x}, {expected_y})."