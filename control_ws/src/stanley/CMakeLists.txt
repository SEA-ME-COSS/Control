cmake_minimum_required(VERSION 3.5)
project(stanley)

# 기본 C++20 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 패키지 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

include(ExternalProject)
# ExternalProject_Add을 사용하여 dbcppp 라이브러리 빌드
ExternalProject_Add(dbcppp_build
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/stanley/dbcppp
  BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/stanley/dbcppp/build
  CONFIGURE_COMMAND ${CMAKE_COMMAND} ..
  BUILD_COMMAND ${MAKE}
  INSTALL_COMMAND ""
  TEST_COMMAND ""
)

include_directories(
  include/${PROJECT_NAME}
  include/stanley/dbcppp/include # dbcppp 헤더 파일 위치
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/control.cpp
  src/controller/stanley.cpp
)

add_dependencies(${PROJECT_NAME} dbcppp_build) # dbcppp가 먼저 빌드되도록 의존성 추가

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  nav_msgs 
  tf2 
  tf2_geometry_msgs 
  geometry_msgs 
  example_interfaces
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
