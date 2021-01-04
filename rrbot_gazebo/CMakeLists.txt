cmake_minimum_required(VERSION 2.8.3)
project(rrbot_gazebo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  pcl_conversions
  pcl_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

find_package(PCL REQUIRED) # This includes all modules
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

#warning: variable CMAKE_CXX_FLAGS is modified
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#warning: use of link_directories() is strongly discouraged
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${PCL_DEFINITIONS}
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
    pcl_conversions
    pcl_ros
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} src/model_push.cpp)
target_link_libraries(
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)

add_executable(conveyor_tf2_broadcast src/conveyor_tf2_broadcast.cpp)
target_link_libraries(conveyor_tf2_broadcast ${catkin_LIBRARIES})

add_executable(periodic_snapshotter src/periodic_snapshotter.cpp)
target_link_libraries(periodic_snapshotter ${catkin_LIBRARIES})

add_executable(pcl_filter src/pcl_filter.cpp)
target_link_libraries(pcl_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(DIRECTORY worlds
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
