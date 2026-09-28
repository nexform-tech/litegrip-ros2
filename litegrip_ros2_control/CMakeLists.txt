cmake_minimum_required(VERSION 3.16)
project(litegrip_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# The SDK this component drives. It is a plain-cmake package (litegrip_cpp), not
# an ament one, and is found through the normal CMake package mechanism — so a
# colcon workspace containing both packages works, as does an installed prefix.
find_package(litegrip_cpp REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# The ros2_control hardware interface plugin
#
# ★ This is now a THIN SHELL over litegrip_cpp::ControlLoop.
#
# It used to bridge to a separate Python daemon over a seqlock shared-memory
# segment, because a ros2_control hardware component must be a C++ pluginlib
# plugin while the SDK was Python-only. That constraint is gone, so the plugin
# links the SDK directly: no second process, no shared memory, no Python in the
# stack, and no ctypes layout that has to be kept in sync by hand.
# ─────────────────────────────────────────────────────────────────────────────
add_library(litegrip_ros2_control SHARED src/litegrip_system.cpp)
target_include_directories(litegrip_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(litegrip_ros2_control litegrip_cpp::litegrip_cpp)
ament_target_dependencies(litegrip_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface litegrip_ros2_control.xml)

# ─────────────────────────────────────────────────────────────────────────────
# Installation
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS litegrip_ros2_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(litegrip_cpp)

# ─────────────────────────────────────────────────────────────────────────────
# Tests
#
# ament_cmake_gtest drives the component's lifecycle directly (not through
# pluginlib) with a synthetic HardwareInfo, all of it in dry_run so no hardware
# is touched. That is what makes parameter parsing, the unit conversion and the
# read/write path verifiable without a gripper.
# ─────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_litegrip_system test/test_litegrip_system.cpp)
  target_link_libraries(test_litegrip_system litegrip_ros2_control)
  # Point the test at the SDK's installed safety baseline by absolute path, so
  # it does not depend on the working directory. litegrip_cpp exports this
  # location from its CMake package.
  target_compile_definitions(test_litegrip_system PRIVATE
    LITEGRIP_TEST_BASELINE="${litegrip_cpp_DATA_DIR}/safety_limits_350.json")
endif()

ament_package()
