cmake_minimum_required(VERSION 3.8)
project(cabot_odrive_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(odrive_can REQUIRED)

include_directories(include)
include_directories(../odrive_base/include)
include_directories(../odrive_node/include)

#add_executable(odrive_can_gain_setter
#  ../odrive_base/src/epoll_event_loop.cpp
#  ../odrive_base/src/socket_can.cpp
#  ../odrive_node/src/arbitrary_parameter.cpp
#  ../odrive_node/src/demangle.cpp
#  src/odrive_can_gain_setter.cpp
#  src/main_odrive_can_gain_setter.cpp
#  include/odrive_can_gain_setter.hpp)
#
#ament_target_dependencies(odrive_can_gain_setter
#  rclcpp
#  odrive_can
#)
#
#target_compile_features(odrive_can_gain_setter PRIVATE cxx_std_20)
#
#install(
#  TARGETS odrive_can_gain_setter
#  DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY json
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
