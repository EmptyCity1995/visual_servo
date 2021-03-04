project(tutorial-ros-pioneer)

cmake_minimum_required(VERSION 2.6)

find_package(visp_ros REQUIRED
             visual_servo)

include_directories( "~/catkin_ws/src/visual_servo/include/"
                    ${visp_ros_INCLUDE_DIRS})

add_executable(tutorial-ros-pioneer-visual-servo tutorial-ros-pioneer-visual-servo.cpp)
target_link_libraries(tutorial-ros-pioneer-visual-servo ${visp_ros_LIBRARIES})



