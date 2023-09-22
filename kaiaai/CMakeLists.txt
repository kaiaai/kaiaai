cmake_minimum_required(VERSION 3.8)
project(kaiaai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
  PROGRAMS script/clone_robot_description.sh
  DESTINATION lib/${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
ament_package()
