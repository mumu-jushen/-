cmake_minimum_required(VERSION 3.0.2)
project(pose_control)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR};/usr/include/eigen3")
endif()

find_package(catkin REQUIRED COMPONENTS
    moveit_ros_planning_interface
    moveit_visual_tools
    sensor_msgs
    roscpp
    rospy
    std_msgs
    franka_gripper
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs moveit_visual_tools moveit_ros_planning_interface 
    DEPENDS
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(pose_control src/moveit_test.cpp)
target_link_libraries(pose_control
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


add_executable(gripper_close_control src/gripper_close_control.cpp)
target_link_libraries(gripper_close_control
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(gripper_open_control src/gripper_open_control.cpp)
target_link_libraries(gripper_open_control
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(joint_test src/joint_test.cpp)
target_link_libraries(joint_test
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(calib_motion src/calib_motion.cpp)
target_link_libraries(calib_motion
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(grasp_test src/grasp_test.cpp)
target_link_libraries(grasp_test
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(franka_grasp src/franka_grasp.cpp)
target_link_libraries(franka_grasp
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


add_executable(tf_tree src/tf_tree.cpp)
target_link_libraries(tf_tree
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)










