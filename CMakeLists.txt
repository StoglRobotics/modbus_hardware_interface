cmake_minimum_required(VERSION 3.8)
project(modbus_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(PkgConfig REQUIRED)  # Find pkg-config module
# Use pkg-config to get information about libmodbus
pkg_check_modules(LIBMODBUS REQUIRED IMPORTED_TARGET libmodbus)

# Export addtionaly libmodbus modules
# set(_exported_dependencies "LIBMODBUS")

# find_package(ament_cmake_libraries QUIET REQUIRED)
#
# if(NOT _exported_dependencies STREQUAL "")
#   find_package(ament_cmake_core QUIET REQUIRED)
#   set(Modbus_hardware_interface_DEPENDENCIES ${_exported_dependencies})
#   set(Modbus_hardware_interface_RECURSIVE_DEPENDENCIES ${_exported_dependencies})
#   set(_libraries)
#   foreach(_dep ${_exported_dependencies})
#     if(NOT ${_dep}_FOUND)
#       find_package("${_dep}" QUIET)
#     endif()
#     if(NOT ${_dep}_FOUND)
#       pkg_check_modules(${_dep} IMPORTED_TARGET)
#     endif()
#     if(NOT ${_dep}_FOUND)
#       find_package("${_dep}" REQUIRED)
#     endif()
#   endforeach()
# endif()

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
target_link_libraries(modbus_hardware_interface PkgConfig::LIBMODBUS)
ament_target_dependencies(modbus_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})
# set_target_properties(modbus_hardware_interface PROPERTIES
#   INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
#   INTERFACE_LINK_LIBRARIES "PkgConfig::LIBMODBUS"
# )

# modbus mock server
add_executable(modbus_mock_server_node src/modbus_mock_server_node.cpp)
target_link_libraries(modbus_mock_server_node PkgConfig::LIBMODBUS)
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
