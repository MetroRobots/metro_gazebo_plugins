cmake_minimum_required(VERSION 3.5)
project(gazebo_base2d_plugin)

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

find_package(ament_cmake REQUIRED)
find_package(base2d_kinematics REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(gazebo_base2d_plugin SHARED src/gazebo_base2d_plugin.cpp)
target_include_directories(gazebo_base2d_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(gazebo_base2d_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)
ament_target_dependencies(gazebo_base2d_plugin
  base2d_kinematics
  geometry_msgs
  nav_msgs
  nav_2d_utils
  tf2_ros
  rclcpp
)

ament_export_libraries(gazebo_base2d_plugin)
ament_export_targets(export_gazebo_base2d_plugin)
install(TARGETS gazebo_base2d_plugin
        EXPORT export_gazebo_base2d_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)

ament_package()
