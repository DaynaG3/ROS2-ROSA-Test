#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String

class InteractiveServiceNode(Node):
    def __init__(self):
        super().__init__('interactive_service_node')

        # Create a service to handle string input
        self.service = self.create_service(SetBool, 'send_input', self.handle_input)

        # Create a publisher to send the string to another node
        self.publisher = self.create_publisher(String, 'input_topic', 10)

        self.get_logger().info('InteractiveServiceNode is ready.')

    def handle_input(self, request, response):
        # Request.data is a bool (irrelevant here, but necessary for SetBool type)
        self.get_logger().info('Waiting for input from the terminal...')

        # Read input from terminal
        user_input = input("Enter a message to send: ")

        # Publish the input string to the topic
        msg = String()
        msg.data = user_input
        self.publisher.publish(msg)

        self.get_logger().info(f"Published message: {user_input}")

        # Respond to the service call
        response.success = True
        response.message = "Input received and sent."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down InteractiveServiceNode...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
