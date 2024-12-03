#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>

#include <stdlib.h>
#include <iostream>
#include <array>
#include <cmath>
#include <functional>
#include <stdio.h>
#include <queue>
#include <thread>
#include <time.h>

#include <franka_gripper/GraspActionGoal.h>

class panda_control
{

private:
  Eigen::Matrix<double, 7, 1> arm_joint_states;
  std::thread* force_control_thread;
  ros::NodeHandle node;
  ros::Subscriber joint_data_sub;

public:

  panda_control()
  {
     tf::TransformListener listener;
     tf::StampedTransform transform;

     arm_joint_states.setZero();

     // 力控的线程
     force_control_thread = new std::thread(boost::bind(&panda_control::force_control_func,this));
     // ROS的线程
     joint_data_sub = node.subscribe("/joint_states", 100, &panda_control::jointCallback, this);
  }
 ~panda_control(){}

void jointCallback(const sensor_msgs::JointState::ConstPtr&msg)
{
  arm_joint_states(0, 0) = msg->position[0];
  arm_joint_states(1, 0) = msg->position[1];
  arm_joint_states(2, 0) = msg->position[2];
  arm_joint_states(3, 0) = msg->position[3];
  arm_joint_states(4, 0) = msg->position[4];
  arm_joint_states(5, 0) = msg->position[5];
  arm_joint_states(6, 0) = msg->position[6];
//  std::cout << arm_joint_states << std::endl;
}

void force_control_func(void)
{
    // MoveIt 使用JointModelGroup储存机械臂的joints，被称为PLANNING_GROUP.在整个运动中"planning group"和"joint model group"可以互换使用
    static const std::string PLANNING_GROUP = "panda_arm";

    // 通过 创建planning_interface:`MoveGroupInterface` 类的实例可以轻松连接、控制或者计划planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // 获取机械臂状态
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    move_group.setMaxAccelerationScalingFactor(0.2);

//  //  // MoveItVisualTools包提供了很多在RVIZ下可用的可视化的物体、机器人和轨迹以及调试工具
//    namespace rvt = rviz_visual_tools;
//    moveit_visual_tools::MoveItVisualTools visual_tools("world");
//    visual_tools.deleteAllMarkers();

//    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//    text_pose.translation().z() = 1.0;

//    // 我们可以计划一个在这个group的运动去获得末端的位姿
//    geometry_msgs::Pose target_pose1;
//    target_pose1.orientation.x = 1.0;
//    //target_pose1.orientation.w = 0.5;
//    target_pose1.position.x = 0.18;
//    target_pose1.position.y = -0.4;
//    target_pose1.position.z = 0.6;
//    move_group.setPoseTarget(target_pose1);

//    // 现在，我们call planner来计算计划并将其可视化。 请注意，我们只是在计划，而不是要求move_group实际移动机器人。
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//    bool success;
//    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//    // 我们也可以把这个计划在Rviz中用一条带有标记的线来进行可视化。
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//    visual_tools.publishAxisLabeled(target_pose1, "pose1");
//    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//    visual_tools.trigger();
//    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//    move_group.move();

//    sleep(2.0);

    // ---------------关节角度控制--------------------
    std::vector<double> joint_group_positions;
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//    joint_group_positions[0] = 2.65;
//    joint_group_positions[1] = 0.436;
//    joint_group_positions[2] = 2.60;
//    joint_group_positions[3] = -2.0;
//    joint_group_positions[4] = -0.215;
//    joint_group_positions[5] = 1.67;
//    joint_group_positions[6] = 0;

    std::cout << "arm_joint_states" << arm_joint_states << std::endl;
    joint_group_positions[0] = arm_joint_states(0, 0);
    joint_group_positions[1] = arm_joint_states(1, 0);
    joint_group_positions[2] = arm_joint_states(2, 0);
    joint_group_positions[3] = arm_joint_states(3, 0);
    joint_group_positions[4] = arm_joint_states(4, 0);
    joint_group_positions[5] = arm_joint_states(5, 0);
    joint_group_positions[6] = arm_joint_states(6, 0)+0.2;

    move_group.setJointValueTarget(joint_group_positions);

    bool success;
    success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();
}

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_test");

  // 在CPU中开启一个线程
  ros::AsyncSpinner spinner(1);
  spinner.start();
  panda_control controller;
  std::cout << "ok" << std::endl;
  ros::waitForShutdown();
  return 0;
}

