cmake_minimum_required(VERSION 3.22)
project(ur5e_hande_moveit_config)

find_package(ament_cmake REQUIRED)

# We REMOVED the 'install(PROGRAMS ...)' section because 
# manual_pose_move.py now lives in ur5e_task_control.

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()