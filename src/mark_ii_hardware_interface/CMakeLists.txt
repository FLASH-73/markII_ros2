cmake_minimum_required(VERSION 3.8)
project(mark_ii_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Create the shared library for the hardware interface
add_library(${PROJECT_NAME} SHARED
  src/mark_ii_hardware_interface.cpp
)

# Add include directories for this package and for Python
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${Python3_INCLUDE_DIRS}
)

# Link pybind11 and other ROS 2 libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
  Python3::Python
  ${pybind11_vendor_LIBRARIES}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
)

# make the absolute path to libpython3.x.so available to your code
target_compile_definitions(${PROJECT_NAME} PRIVATE
  PYTHON_LIBRARY_PATH="${Python3_LIBRARY}"
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_python_install_package(mark_ii_hardware_interface)


# Install the compiled C++ library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---- THIS IS THE CRITICAL CHANGE ----
# Explicitly export the plugin description file so pluginlib can find it
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# Install config and launch files
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()