cmake_minimum_required(VERSION 3.5)
project(turtle_agent)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  scripts
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  scripts/tools
  DESTINATION lib/${PROJECT_NAME})  

# Install Python nodes
install(
  PROGRAMS
    scripts/turtle_agent.py  # Update with your actual script paths
    scripts/help.py  # Update with your actual script paths
    scripts/llm.py  # Update with your actual script paths
    scripts/prompts.py  # Update with your actual script paths
    scripts/service_node.py  # Update with your actual script paths
  DESTINATION lib/${PROJECT_NAME}
)

# Add ament package
ament_package()
