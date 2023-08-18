cmake_minimum_required(VERSION 3.5)
project(gazebo_collision_plugin)

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
find_package(builtin_interfaces REQUIRED)
find_package(collision_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(gazebo_collision SHARED src/gazebo_collision_plugin.cpp)
target_include_directories(gazebo_collision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gazebo_collision
  gazebo_dev
  builtin_interfaces
  gazebo_ros
  collision_msgs
)

ament_export_libraries(gazebo_collision)
ament_export_targets(export_gazebo_collision)
install(TARGETS gazebo_collision
        EXPORT export_gazebo_collision
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)

ament_package()
