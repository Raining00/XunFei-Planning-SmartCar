project(informed_rrt_star_planner)
set(CMAKE_BUILD_TYPE Release)
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto InformedRRTStarPlannerConfigProtoSrc InformedRRTStarPlannerConfigProtoHds ${ProtoFiles})


set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

find_package(Eigen3 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
link_libraries(${OMPL_LIBRARY_DIRS})

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${OMPL_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  SHARED
  ${InformedRRTStarPlannerConfigProtoSrc}
  ${InformedRRTStarPlannerConfigProtoHds}
  informed_rrt_star.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    roborts_costmap
    ${OMPL_LIBRARIES}
)
add_library(planning::global_planner::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
