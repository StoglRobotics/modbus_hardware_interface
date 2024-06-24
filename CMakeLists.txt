cmake_minimum_required(VERSION 3.8)
project(modbus_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(PkgConfig REQUIRED)  # Find pkg-config module
# Use pkg-config to get information about libmodbus
pkg_check_modules(LIBMODBUS REQUIRED IMPORTED_TARGET libmodbus)

# define ROS Package dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclpy
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# modbus hardware interface
add_library(
  modbus_hardware_interface
  SHARED
  src/modbus_hardware_interface.cpp
)
target_include_directories(
  modbus_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/modbus_hardware_interface>
)
target_link_libraries(modbus_hardware_interface ${LIBMODBUS_LIBRARIES})
ament_target_dependencies(modbus_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

# modbus mock server
add_executable(modbus_mock_server_node src/modbus_mock_server_node.cpp)
target_link_libraries(modbus_mock_server_node ${LIBMODBUS_LIBRARIES})
ament_target_dependencies(modbus_mock_server_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  hardware_interface modbus_hardware_interface.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_modbus_hardware_interface test/test_modbus_hardware_interface.cpp)
  target_include_directories(test_modbus_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_modbus_hardware_interface
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    ros2_control_test_assets
  )

endif()

install(
  DIRECTORY include/
  DESTINATION include/modbus_hardware_interface
)

install(
  TARGETS modbus_hardware_interface
  EXPORT export_modbus_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS modbus_mock_server_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_modbus_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
