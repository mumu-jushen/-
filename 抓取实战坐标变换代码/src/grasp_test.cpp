#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>

#include <franka_gripper/GraspActionGoal.h>

#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_test");
  ros::NodeHandle node;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  Eigen::Vector3d init_position;
  Eigen::Quaterniond init_rotation;

  init_position.setZero();
  init_rotation.setIdentity();

  // 抓手设置
  actionlib::SimpleActionClient<franka_gripper::MoveAction> ac_move("/franka_gripper/move", true);
  actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp("/franka_gripper/grasp", true);

  ROS_INFO("Waiting for MoveAction server to start.");
  ROS_INFO("Waiting for GraspAction server to start.");
  ac_move.waitForServer();
  ac_grasp.waitForServer();
  franka_gripper::MoveGoal move_goal;
  franka_gripper::GraspGoal grasp_goal;

  // tf_tree
 try
  {
     listener.waitForTransform("/world", "/panda_link8", ros::Time(0), ros::Duration(10.0));
     listener.lookupTransform("/world", "/panda_link8", ros::Time(0), transform);
  }
 catch (tf::TransformException ex)
  {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

    init_position(0, 0) = transform.getOrigin().x();
    init_position(1, 0) = transform.getOrigin().y();
    init_position(2, 0) = transform.getOrigin().z();
    init_rotation.x() = transform.getRotation().getX();
    init_rotation.y() = transform.getRotation().getY();
    init_rotation.z() = transform.getRotation().getZ();
    init_rotation.w() = transform.getRotation().getW();

    std::cout << transform.getOrigin().x() << std::endl;
    std::cout << transform.getOrigin().y() << std::endl;
    std::cout << transform.getOrigin().z() << std::endl;
    std::cout << transform.getRotation().getX() << std::endl;
    std::cout << transform.getRotation().getY() << std::endl;
    std::cout << transform.getRotation().getZ() << std::endl;
    std::cout << transform.getRotation().getW() << std::endl;

  // 线程开始
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;

  // --------移动至抓取点------------ 

  // 我们可以计划一个在这个group的运动去获得末端的位姿
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = -0.0771648;
  target_pose1.position.y = -0.693497;
  target_pose1.position.z = 0.104803;
  target_pose1.orientation.x = -0.39387;
  target_pose1.orientation.y = 0.918869;
  target_pose1.orientation.z = -0.00171837;
  target_pose1.orientation.w = -0.0232962;
  move_group.setPoseTarget(target_pose1);

  // 现在，我们call planner来计算计划并将其可视化。 请注意，我们只是在计划，而不是要求move_group实际移动机器人。
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // 我们也可以把这个计划在Rviz中用一条带有标记的线来进行可视化。
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

  // --------闭合抓手------------ 
  std::cout << "Closing gripper" << std::endl;

   grasp_goal.width = 0.0;
   grasp_goal.speed = 0.05;
   grasp_goal.force = 100;
   grasp_goal.epsilon.inner = 0.2;
   grasp_goal.epsilon.outer = 0.2;
   ac_grasp.sendGoal(grasp_goal);
      
   //wait for the action to return
  success = ac_grasp.waitForResult(ros::Duration(10.0));
    
  std::cout << "Result: " << success << std::endl;


  // --------抬起至等候位------------ 
  geometry_msgs::Pose target_pose2;
  target_pose2.position.x = -0.0782;
  target_pose2.position.y = -0.5494;
  target_pose2.position.z = 0.46142;
  target_pose2.orientation.x = -0.38768;
  target_pose2.orientation.y = 0.92158;
  target_pose2.orientation.z = -0.01723;
  target_pose2.orientation.w = -0.0099;
  move_group.setPoseTarget(target_pose2);

  moveit::planning_interface::MoveGroupInterface::Plan lift_plan;

  success = (move_group.plan(lift_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // 我们也可以把这个计划在Rviz中用一条带有标记的线来进行可视化。
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(lift_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

  // --------移动至放置点------------ 

  geometry_msgs::Pose target_pose3;
  target_pose3.position.x = -0.34793;
  target_pose3.position.y = -0.64352;
  target_pose3.position.z = 0.17033;
  target_pose3.orientation.x = -0.36833;
  target_pose3.orientation.y = 0.92918;
  target_pose3.orientation.z = 0.00808;
  target_pose3.orientation.w = -0.0298;
  move_group.setPoseTarget(target_pose3);

  moveit::planning_interface::MoveGroupInterface::Plan place_plan;

  success = (move_group.plan(place_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // 我们也可以把这个计划在Rviz中用一条带有标记的线来进行可视化。
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose3, "pose3");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(place_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

  // --------张开抓手------------ 
  std::cout << "Opening gripper" << std::endl;

  move_goal.width = 0.08;
  move_goal.speed = 0.1;
  ac_move.sendGoal(move_goal);
      
   //wait for the action to return
  success = ac_move.waitForResult(ros::Duration(10.0));      
    
  std::cout << "Result: " << success << std::endl;

  // --------归位至等候位------------ 
  geometry_msgs::Pose target_pose4;
  target_pose4.position.x = -0.0782;
  target_pose4.position.y = -0.5494;
  target_pose4.position.z = 0.46142;
  target_pose4.orientation.x = -0.38768;
  target_pose4.orientation.y = 0.92158;
  target_pose4.orientation.z = -0.01723;
  target_pose4.orientation.w = -0.0099;
  move_group.setPoseTarget(target_pose4);

  moveit::planning_interface::MoveGroupInterface::Plan reset_plan;

  success = (move_group.plan(reset_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // 我们也可以把这个计划在Rviz中用一条带有标记的线来进行可视化。
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose4, "pose4");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(reset_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();


  ros::waitForShutdown();
  return 0;
}















