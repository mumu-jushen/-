#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "gripper_close_control");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");

  actionlib::SimpleActionClient<franka_gripper::MoveAction> ac_move("/franka_gripper/move", true);
  actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp("/franka_gripper/grasp", true);

  ROS_INFO("Waiting for MoveAction server to start.");
  ROS_INFO("Waiting for GraspAction server to start.");
  ac_move.waitForServer();
  ac_grasp.waitForServer();
  franka_gripper::MoveGoal move_goal;
  franka_gripper::GraspGoal grasp_goal;

  std::cout << "Closing gripper" << std::endl;

   grasp_goal.width = 0.0;
   grasp_goal.speed = 0.05;
   grasp_goal.force = 0.01;
   grasp_goal.epsilon.inner = 0.2;
   grasp_goal.epsilon.outer = 0.2;
   ac_grasp.sendGoal(grasp_goal);
      
   //wait for the action to return
  bool success = ac_grasp.waitForResult(ros::Duration(10.0));
    
  std::cout << "Result: " << success << std::endl;
  ros::shutdown();
  return 0;
}


