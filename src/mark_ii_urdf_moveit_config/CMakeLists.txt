cmake_minimum_required(VERSION 3.22)
project(mark_ii_urdf_moveit_config)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# This block MUST be present to fix the error
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
