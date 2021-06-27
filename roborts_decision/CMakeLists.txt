project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  )


find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
)





#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )


catkin_package()

# sel_behavior_node
add_executable(sel_behavior_node
  sel_behavior_node.cpp
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/goal_behavior.h
  executor/chassis_executor.cpp
  )

target_link_libraries(sel_behavior_node 
  PRIVATE
  ${TORCH_LIBRARIES}
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  -lzmq
  )


