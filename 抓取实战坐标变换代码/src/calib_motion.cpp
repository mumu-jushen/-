#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>

#include <franka_gripper/GraspActionGoal.h>



int main(int argc, char **argv)
{

  ros::init(argc, argv, "calib_motion");
  ros::NodeHandle node;
  ros::Subscriber joint_data_sub;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  Eigen::Vector3d init_position;
  Eigen::Quaterniond init_rotation;

  init_position.setZero();
  init_rotation.setIdentity();


  // 在CPU中开启一个线程
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt 使用JointModelGroup储存机械臂的joints，被称为PLANNING_GROUP.在整个运动中"planning group"和"joint model group"可以互换使用
  static const std::string PLANNING_GROUP = "panda_arm";

  // 通过 创建planning_interface:`MoveGroupInterface` 类的实例可以轻松连接、控制或者计划planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // 获取机械臂状态
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);


  // ---------------关节角度控制--------------------
  std::vector<double> joint_group_positions;
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  std::cout << "-------The 1st point for calibration-------" << std::endl;

  joint_group_positions[0] = -1.550;
  joint_group_positions[1] = -0.005;
  joint_group_positions[2] = -0.0157;
  joint_group_positions[3] = -2.246;
  joint_group_positions[4] = -0.016;
  joint_group_positions[5] =  2.254;
  joint_group_positions[6] =  0.785;

  move_group.setJointValueTarget(joint_group_positions);

  bool success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();



  std::cout << "-------The 2nd point for calibration-------" << std::endl;

  sleep(5.0);

  joint_group_positions[0] = -1.7580059219623911;
  joint_group_positions[1] = -0.7975115384302639;
  joint_group_positions[2] =  0.7295145556479855;
  joint_group_positions[3] = -2.3747140256969703;
  joint_group_positions[4] =  0.048327815996276;
  joint_group_positions[5] =  1.8375134221145326;
  joint_group_positions[6] =  1.0099772303435537;

  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  std::cout << "-------The 3th point for calibration-------" << std::endl;

  sleep(5.0);

  joint_group_positions[0] = -1.1890067757857148;
  joint_group_positions[1] =  0.793777057388372;
  joint_group_positions[2] =  0.13752134177164546;
  joint_group_positions[3] = -0.9111059958307365;
  joint_group_positions[4] = -0.5300011982851558;
  joint_group_positions[5] =  1.135486638367176;
  joint_group_positions[6] =  1.4799741600202507;

  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();


//  std::cout << "-------The 4th point for calibration-------" << std::endl;
//
//  sleep(3.0);
//
//  joint_group_positions[0] = -1.550;
//  joint_group_positions[1] = -0.005;
//  joint_group_positions[2] = -0.0157;
//  joint_group_positions[3] = -2.246;
//  joint_group_positions[4] = -0.016;
//  joint_group_positions[5] =  2.254;
//  joint_group_positions[6] =  0;
//
//  move_group.setJointValueTarget(joint_group_positions);
//
//  success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  move_group.move();






















  ros::waitForShutdown();
  return 0;
}

