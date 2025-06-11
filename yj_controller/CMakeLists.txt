cmake_minimum_required(VERSION 3.5)
project(yj_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# include 디렉토리 추가
include_directories(
  include
  ${px4_msgs_INCLUDE_DIRS}
)

# 실행 파일 추가
add_executable(yj_controller_node src/main.cpp)
add_executable(optitrack_odometry_subscriber
    src/optitrack_odometry_subscriber.cpp
)
add_executable(rpy_publisher
    src/rpy_publisher.cpp
)

# 실행 파일에 의존성 연결
ament_target_dependencies(yj_controller_node
  rclcpp
  px4_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  Eigen3
)
ament_target_dependencies(optitrack_odometry_subscriber
    rclcpp
    nav_msgs
    Eigen3
)
ament_target_dependencies(rpy_publisher
    rclcpp
    geometry_msgs
    Eigen3
)

# 실행 파일 설치 경로 설정
install(TARGETS yj_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 디렉토리 설치
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# include 디렉토리 설치
install(DIRECTORY include/
  DESTINATION include/
)

# 패키지 설정
ament_package()