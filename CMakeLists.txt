cmake_minimum_required(VERSION 3.0.2)
project(gloc)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED)

find_package(IBEX REQUIRED)
ibex_init_common() # IBEX should have installed this function
message(STATUS "Found IBEX version ${IBEX_VERSION}")

find_package(Eigen3 REQUIRED NO_MODULE)
message(STATUS "Found Eigen3 version ${EIGEN3_VERSION}")

find_package(CODAC REQUIRED)
message(STATUS "Found Codac version ${CODAC_VERSION}")

find_package(GTSAM REQUIRED)
message(STATUS "Found GTSAM version ${GTSAM_VERSION}")

find_package(Boost REQUIRED COMPONENTS serialization thread timer chrono)


catkin_package(
  CATKIN_DEPENDS gazebo_ros geometry_msgs nav_msgs roscpp rospy std_msgs tf 
  DEPENDS PCL
  INCLUDE_DIRS include
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${IBEX_INCLUDE_DIRS}
  ${CODAC_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)
message(STATUS "Found IBEX include directory ${IBEX_INCLUDE_DIRS}") 
message(STATUS "Found CODAC include directory ${CODAC_INCLUDE_DIRS}") 
message(STATUS "Found GTSAM include directory ${GTSAM_INCLUDE_DIR}") 

link_directories(
  ${IBEX_LIBRARIES_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)
message(STATUS "Found IBEX library directory ${IBEX_LIBRARIES}") 
message(STATUS "Found CODAC library directory ${CODAC_LIBRARIES}") 
message(STATUS "Found GTSAM library directory ${GTSAM_LIBRARY_DIRS}") 

add_executable(tubeloc_circle src/tubeloc_circle.cpp)
target_compile_options(tubeloc_circle PUBLIC ${CODAC_CXX_FLAGS})
target_include_directories(tubeloc_circle SYSTEM PUBLIC ${CODAC_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(tubeloc_circle PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${IBEX_LDFLAGS} ${CODAC_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} Ibex::ibex)

add_executable(tubeloc_example src/tubeloc_example.cpp)
target_compile_options(tubeloc_example PUBLIC ${CODAC_CXX_FLAGS})
target_include_directories(tubeloc_example SYSTEM PUBLIC ${CODAC_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(tubeloc_example PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${IBEX_LDFLAGS} ${CODAC_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} Ibex::ibex)

add_executable(gloc_node src/gloc_node.cpp)
target_compile_options(gloc_node PUBLIC ${CODAC_CXX_FLAGS})
target_include_directories(gloc_node SYSTEM PUBLIC ${CODAC_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(gloc_node PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${IBEX_LDFLAGS} ${CODAC_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} Ibex::ibex)
# target_link_libraries(gloc_node PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${IBEX_LDFLAGS} ${CODAC_LIBRARIES})
message(STATUS "Found IBEX ${IBEX_LDFLAGS}") 

add_executable(gtsam_PlanarSLAM src/gtsam_PlanarSLAM.cpp)
target_compile_options(gtsam_PlanarSLAM PUBLIC ${CODAC_CXX_FLAGS})
target_include_directories(gtsam_PlanarSLAM SYSTEM PUBLIC ${CODAC_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(gtsam_PlanarSLAM PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${IBEX_LDFLAGS} ${CODAC_LIBRARIES} gtsam Ibex::ibex ${CODAC_LIBRARIES} Ibex::ibex)