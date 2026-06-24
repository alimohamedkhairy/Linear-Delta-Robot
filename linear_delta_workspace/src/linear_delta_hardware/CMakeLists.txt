cmake_minimum_required(VERSION 3.8)
project(linear_delta_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(std_srvs REQUIRED)

# Find the system library libserial
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(include)

# Create the library
add_library(linear_delta_hardware SHARED src/linear_delta_hardware_plugin.cpp)

# Use target_link_libraries WITHOUT keywords (no PUBLIC/PRIVATE) 
# to stay compatible with ament_target_dependencies
target_link_libraries(linear_delta_hardware ${LIBSERIAL_LIBRARIES})

target_include_directories(linear_delta_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

# ROS-specific dependencies
ament_target_dependencies(linear_delta_hardware
  hardware_interface
  pluginlib
  rclcpp
  std_srvs
)

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# Install rules with matching EXPORT name
install(TARGETS linear_delta_hardware
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# This must match the EXPORT name in the install(TARGETS) section above
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp)

ament_package()