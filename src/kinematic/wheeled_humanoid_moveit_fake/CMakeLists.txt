cmake_minimum_required(VERSION 3.14)
project(wheeled_humanoid_moveit_fake)

find_package(ament_cmake REQUIRED)

ament_package()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/fake_joint_driver.py
  scripts/robot_description_publisher.py
  scripts/attach_object_demo.py
  scripts/attach_object_gui.py
  DESTINATION lib/${PROJECT_NAME}
)
