cmake_minimum_required(VERSION 3.0.2)
project(covered_area)

## Using to generate json file to use with vim and clang
## Only works with catkin_make
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Reset_Area.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/covered_area_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

