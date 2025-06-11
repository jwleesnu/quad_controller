# cmake_minimum_required(VERSION 3.8)
# project(twist_publisher)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# add_executable(twist_publisher_node src/twist_publisher_node.cpp)
# target_include_directories(twist_publisher_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(twist_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS twist_publisher_node
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(twist_publisher)

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
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# include 디렉토리 추가
include_directories(
  include
  ${px4_msgs_INCLUDE_DIRS}
)

# 실행 파일 추가 (소스 파일 이름은 twist_listener_node.cpp)
add_executable(twist_listener_node src/twist_listener_node.cpp)

# 실행 파일에 의존성 연결
ament_target_dependencies(twist_listener_node
  rclcpp
  px4_msgs
)

# 실행 파일 설치 경로 설정
install(TARGETS twist_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 디렉토리 설치 (launch 파일을 따로 생성했다면)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# include 디렉토리 설치 (필요시)
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()