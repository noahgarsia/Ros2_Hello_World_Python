cmake_minimum_required(VERSION 3.8)
project(project_1)

# =============================================
# FIND DEPENDENCIES
# =============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# =============================================
# INCLUDE PATHS
# =============================================
include_directories(include)

# =============================================
# C++ PUBLISHER LIBRARY
# =============================================
add_library(project_1_publisher_lib
  cpp/cpp_minimal_publisher.cpp
)

ament_target_dependencies(project_1_publisher_lib
  rclcpp
  std_msgs
)

add_executable(cpp_minimal_publisher
  cpp/publisher_main.cpp
)

ament_target_dependencies(cpp_minimal_publisher
  rclcpp
  std_msgs
)

target_link_libraries(cpp_minimal_publisher
  project_1_publisher_lib
)

# =============================================
# C++ SUBSCRIBER LIBRARY
# =============================================
add_library(project_1_subscriber_lib
  cpp/cpp_minimal_subscriber.cpp
)

ament_target_dependencies(project_1_subscriber_lib
  rclcpp
  std_msgs
)

add_executable(cpp_minimal_subscriber
  cpp/subscriber_main.cpp
)

ament_target_dependencies(cpp_minimal_subscriber
  rclcpp
  std_msgs
)

target_link_libraries(cpp_minimal_subscriber
  project_1_subscriber_lib
)

# =============================================
# INSTALL EXECUTABLES + HEADERS
# =============================================
install(TARGETS
  cpp_minimal_publisher
  cpp_minimal_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    cpp_test_publisher
    test/gtest/test_publisher.cpp
  )
  target_link_libraries(cpp_test_publisher project_1_publisher_lib)

  ament_add_gtest(
    cpp_test_subscriber
    test/gtest/test_subscriber.cpp
  )
  target_link_libraries(cpp_test_subscriber project_1_subscriber_lib)
endif()

# =============================================
# Install launch scripts
# =============================================
install(PROGRAMS
  launch/cpp_launcher.sh
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================
# EXPORT PACKAGE
# =============================================
ament_package()
