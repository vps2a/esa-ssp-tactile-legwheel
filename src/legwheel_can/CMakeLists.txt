cmake_minimum_required(VERSION 3.15)
project(legwheel_can)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# We only want to use CANdle-SDK as a C++ library inside this ROS package.
# Do not build/install candletool, examples, Python bindings, tests, or udev rules.
set(CANDLESDK_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(CANDLESDK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CANDLESDK_BUILD_CANDLETOOL OFF CACHE BOOL "" FORCE)
set(MAKE_TESTS OFF CACHE BOOL "" FORCE)
set(CANDLE_BUILD_STATIC ON CACHE BOOL "" FORCE)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/CANdle-SDK
  ${CMAKE_CURRENT_BINARY_DIR}/CANdle-SDK
)

add_executable(md80_legwheel_node
  src/md80_legwheel_node.cpp
)

add_executable(md80_zero_node
  src/md80_zero_node.cpp
)

ament_target_dependencies(md80_legwheel_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  std_srvs
)

ament_target_dependencies(md80_zero_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

target_link_libraries(md80_legwheel_node
  candle
  Threads::Threads
)

target_link_libraries(md80_zero_node
  candle
  Threads::Threads
)

target_compile_features(md80_legwheel_node PUBLIC cxx_std_20)
target_include_directories(md80_legwheel_node PRIVATE include)
target_compile_features(md80_zero_node PUBLIC cxx_std_20)

install(TARGETS
  md80_legwheel_node
  md80_zero_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/legwheel_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME legwheel_controller_node
)

install(PROGRAMS
  scripts/wheel_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_controller_node
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_wheel_velocity_governor test/test_wheel_velocity_governor.cpp)
  target_include_directories(test_wheel_velocity_governor PRIVATE include)
  add_test(NAME wheel_velocity_governor COMMAND test_wheel_velocity_governor)
endif()

ament_package()
