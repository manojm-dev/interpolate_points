cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)

# Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

# Add library
add_library(${PROJECT_NAME}
  src/catmull_rom_spline.cpp
)

# Add executable
add_executable(${PROJECT_NAME}_node
  src/path_publisher_node.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# Install targets
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY rviz launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

