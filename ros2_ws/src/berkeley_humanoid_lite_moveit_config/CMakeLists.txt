cmake_minimum_required(VERSION 3.8)
project(berkeley_humanoid_lite_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/sim_trajectory_server.py
  scripts/check_ik.py
  scripts/motion_commander.py
  scripts/hardware_bridge.py
  scripts/left_hand_adapter.py
  scripts/drag_teleop.py
  scripts/arm_panel.py
  scripts/robot_view.py
  scripts/finger_panel.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
