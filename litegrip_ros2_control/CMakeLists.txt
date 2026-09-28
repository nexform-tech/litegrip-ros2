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
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# The shared memory contract library
#
# The C++ plugin links against it directly; the Python daemon calls the same
# .so through ctypes. The layout is defined in exactly one place,
# include/litegrip_ros2_control/litegrip_shm.h, and the two sides cross-check
# their sizes when the shared library is loaded (see shm_bridge._verify_sizes),
# while the field offsets are cross-checked by test/test_shm_layout.py using
# litegrip_shm_layout_probe.
# ─────────────────────────────────────────────────────────────────────────────
add_library(litegrip_shm SHARED src/litegrip_shm.cpp)
target_include_directories(litegrip_shm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(litegrip_shm PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ─────────────────────────────────────────────────────────────────────────────
# The ros2_control hardware interface plugin
#
# The same paradigm as the arm (litearm_ros2_control): the plugin only does
# memcpy + unit conversion, and every bit of hardware interaction lives in the
# Python daemon (over shared memory).
# ─────────────────────────────────────────────────────────────────────────────
add_library(litegrip_ros2_control SHARED src/litegrip_system.cpp)
target_include_directories(litegrip_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(litegrip_ros2_control litegrip_shm)
ament_target_dependencies(litegrip_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface litegrip_ros2_control.xml)

# ─────────────────────────────────────────────────────────────────────────────
# Layout probe: exports struct size/offsetof for the Python-side tests
# ─────────────────────────────────────────────────────────────────────────────
add_executable(litegrip_shm_layout_probe test/litegrip_shm_layout_probe.cpp)
target_link_libraries(litegrip_shm_layout_probe litegrip_shm)

# ─────────────────────────────────────────────────────────────────────────────
# Installation
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS litegrip_shm litegrip_ros2_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS litegrip_shm_layout_probe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

# ─────────────────────────────────────────────────────────────────────────────
# Python package (shared memory bindings + hardware daemon + the ported safety
# layer)
#
# ★ The daemon carries **no ROS dependency**: it only reads shared memory and
#   calls the SDK. This is isomorphic to the arm, and the benefit is that the
#   real-time side has no Python at all and this process runs without rclpy.
# ─────────────────────────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

install(PROGRAMS scripts/litegrip_hw_daemon
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(litegrip_shm)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
