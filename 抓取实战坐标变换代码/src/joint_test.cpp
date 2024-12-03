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

  ros::init(argc, argv, "moveit_test");
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

//  // MoveItVisualTools包提供了很多在RVIZ下可用的可视化的物体、机器人和轨迹以及调试工具
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;

  // 我们可以计划一个在这个group的运动去获得末端的位姿
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 1.0;
  target_pose1.position.x = 0.0;
  target_pose1.position.y = -0.5224;
  target_pose1.position.z = 0.3688;
  move_group.setPoseTarget(target_pose1);

  // 现在，我们call planner来计算计划并将其可视化。 请注意，我们只是在计划，而不是要求move_group实际移动机器人。
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // 我们也可以把这个计划在Rviz中用一条带有标记的线来进行可视化。
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  //move_group.move();


  // ---------------关节角度控制--------------------
  std::vector<double> joint_group_positions;
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = -0.012241454241334828; //-0.004755951056354687;
  joint_group_positions[1] = -0.395629225915397; // -0.7816322353346304;
  joint_group_positions[2] = 0.026243317118807803; //0.0027716900604989447;
  joint_group_positions[3] = -2.097061959927875;//-2.357223240506345;
  joint_group_positions[4] = -0.005668849258472385;//-0.00609153702422068;
  joint_group_positions[5] =  1.671259904729217;//1.5533100152280594;
  joint_group_positions[6] =  0.7950523641308148;//0.788903862893995;
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  ros::waitForShutdown();
  return 0;
}

