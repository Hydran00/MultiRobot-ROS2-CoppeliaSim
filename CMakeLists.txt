cmake_minimum_required(VERSION 3.16)
project(ur_coppeliasim LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  ur_coppeliasim
  SHARED
  hardware/HWInterface.cpp
)
target_compile_features(ur_coppeliasim PUBLIC cxx_std_17)
target_include_directories(ur_coppeliasim PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ur_coppeliasim>
)
ament_target_dependencies(
  ur_coppeliasim PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_1_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ur_coppeliasim.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ur_coppeliasim
)

install(
  DIRECTORY launch urdf meshes config rviz
  DESTINATION share/ur_coppeliasim
)

install(TARGETS ur_coppeliasim
  EXPORT export_ur_coppeliasim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ur_coppeliasim HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
