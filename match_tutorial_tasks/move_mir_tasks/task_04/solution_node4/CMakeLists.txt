# This CMakeLists.txt is for the C++ example

cmake_minimum_required(VERSION 3.0.2)

project(solution_node4)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs nav_msgs geometry_msgs move_base_msgs actionlib roscpp
)

generate_messages(
   DEPENDENCIES
   nav_msgs
   geometry_msgs
 )
 
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/solution_04.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)