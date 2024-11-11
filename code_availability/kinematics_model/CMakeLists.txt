cmake_minimum_required(VERSION 2.8.3)
project(kinematics_model)
add_compile_options(-std=c++11)

   
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
   roscpp
   visualization_msgs
   tf
   roslib
   std_msgs
   message_generation
   interactive_markers
  
)
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp roslib message_runtime std_msgs
   LIBRARIES kinematics_model
)


include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp library 
add_library(
${PROJECT_NAME} 
src/model_visualization.cpp 
)

 
add_executable(model_visualization src/model_visualization.cpp)
target_link_libraries(model_visualization kinematics_model ${catkin_LIBRARIES})
 


