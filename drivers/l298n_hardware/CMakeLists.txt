cmake_minimum_required(VERSION 3.10)
project(l298n_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/l298n_system.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

# pigpio library
# Link against pigpiod_if2 client library (daemon-based)
find_library(PIGPIOD_IF2_LIB pigpiod_if2)
if(NOT PIGPIOD_IF2_LIB)
  message(FATAL_ERROR "pigpiod_if2 library not found. Install 'libpigpiod-if-dev'.")
endif()

target_link_libraries(${PROJECT_NAME}
  ${PIGPIOD_IF2_LIB}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface l298n_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES l298n_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
