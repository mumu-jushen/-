#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/transform_listener.h>
#include <franka_gripper/GraspActionGoal.h>

//         gripper
#include <iostream>
#include <sstream>
#include <string>
#include <franka/exception.h>
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>
#include <termio.h>
#include <stdio.h>
#include <sstream>
#include <cmath>
using namespace std;


int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);

    in = getchar();

    tcsetattr(0,TCSANOW,&stored_settings);
    return in;
}

double grasp_angle;
void object_pose_Callback(const std_msgs::Float64MultiArray &object_tmp)
{
    grasp_angle = object_tmp.data[3]; //rad
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "franka_grasp");
  ros::NodeHandle node;

  // ##################################################### gripper init  #####################################################
  actionlib::SimpleActionClient<franka_gripper::MoveAction> ac_move("/franka_gripper/move", true);
  actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp("/franka_gripper/grasp", true);
  // ##################################################### gripper init  #####################################################

  ac_move.waitForServer();
  ac_grasp.waitForServer();
  franka_gripper::MoveGoal move_goal;
  franka_gripper::GraspGoal grasp_goal;

  // ##################################################### open gripper   #####################################################
  std::cout << "Opening gripper" << std::endl;
  move_goal.width = 1.0;
  move_goal.speed = 0.1;
  ac_move.sendGoal(move_goal);
   //wait for the action to return
  bool SuccessGripper = ac_move.waitForResult(ros::Duration(10.0));
  std::cout << "gripper Result: " << SuccessGripper << std::endl;
  // ##################################################### open gripper   #####################################################


  //  ##################################################### read the grasp angle   ############################################
   ros::Subscriber sub = node.subscribe("object_pose", 1000, object_pose_Callback);
   ros::Rate loop_rate(10);
   int Keyboard=0;
   tf::TransformListener listener;
     //  ##################################################### read the grasp angle   ############################################

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


    while (ros::ok()){

        // read grasp pose from tf tree
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/panda_link0", "/grasppoint", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/panda_link0", "/grasppoint", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        double grasp_x = transform.getOrigin().getX();
        double grasp_y = transform.getOrigin().getY();
        double grasp_z = transform.getOrigin().getZ() + 0.140;   // 机械臂末端到抓手底部的距离
        double qx = transform.getRotation().x(), qy = transform.getRotation().y(),
                qz = transform.getRotation().z(), qw = transform.getRotation().w();
        double grasp_angle2 = grasp_angle;
        std::cout<<">>>>>>>>>>>: "<< grasp_angle2 << std::endl;


//        double grasp_angle2 = 0;
//        double grasp_x = -0.0105;
//        double grasp_y = -0.7038;
//        double grasp_z = 0.1509;
//        double qx = -0.3717;
//        double qy = 0.9282;
//        double qz = 0;
//        double qw = 0;
//        std::cout<<111111111111111111<< std::endl;



        // wait for my agree
        Keyboard =scanKeyboard();

        if (Keyboard == 32){

            //-----------------------------------------------------  gripper:   open and rotaion  ------------------------------------------------------
            std::cout << "Opening gripper" << std::endl;

            move_goal.width = 0.025;
            move_goal.speed = 0.1;
            ac_move.sendGoal(move_goal);
             //wait for the action to return
            SuccessGripper = ac_move.waitForResult(ros::Duration(10.0));
            std::cout << "gripper Result: " << SuccessGripper << std::endl;

            // ---------------关节角度控制--------------------
//            std::vector<double> joint_group_positions;
//            moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
//            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//            joint_group_positions[6] = 90 + ( grasp_angle2/3.1415926 ) * 180 ;    ///  旋转最后一个关节，保证抓取角度
//            joint_group_positions[6] = M_PI/2 +  grasp_angle2;
//            move_group.setJointValueTarget(joint_group_positions);
//            bool success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//            move_group.move();
//
            std::cout<<'grasp_angle2: '<<grasp_angle2<< std::endl;

            geometry_msgs::Pose target_pose1;
            target_pose1 = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
//            std::cout<< target_pose1<< std::endl;

//            sensor_msgs::JointState current_joint_value;
//            current_joint_value = move_group.getCurrentJointValues();
//            std::cout<< current_joint_value << std::endl;


            //-----------------------------------------------------  move franka to grasp  ------------------------------------------------------
//            geometry_msgs::Pose target_pose1;
            target_pose1.position.x = grasp_x;
            target_pose1.position.y = grasp_y;
            target_pose1.position.z = grasp_z ;  // 先到预抓取位置（比抓取位置高0.05米），保证夹抓下降路径不会碰到零件
//            target_pose1.orientation.x = qx;
//            target_pose1.orientation.y = qy;
//            target_pose1.orientation.z = qz;
//            target_pose1.orientation.w = qw;
            std::cout << "Pregrasp point x:" << target_pose1.position.x << std::endl;
            std::cout << "Pregrasp point y:" << target_pose1.position.y << std::endl;
            std::cout << "Pregrasp point z:" << target_pose1.position.z << std::endl;


            std::cout << "Wait for grasping" << std::endl;
            move_group.setPoseTarget(target_pose1);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
            move_group.setMaxVelocityScalingFactor(0.3);
            move_group.setMaxAccelerationScalingFactor(0.3);
            move_group.move();


            std::vector<double> joint_group_positions;
            moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

            //  旋转最后一个关节，保证抓取角度
            ROS_INFO("rotation the gripper .....");
//            joint_group_positions[6] = M_PI/2 +  grasp_angle2;
            joint_group_positions[6] =  M_PI/4 + (-1) * grasp_angle2 ;
            std::cout << joint_group_positions[6] << std::endl;
            move_group.setJointValueTarget(joint_group_positions);
            success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();


            geometry_msgs::Pose target_pose2;
            target_pose2 = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
            std::cout<< target_pose2<< std::endl;

            //-----------------------------------------------------  move franka to grasp  ------------------------------------------------------
            target_pose2.position.z = grasp_z ;  // 到抓取位置
            move_group.setPoseTarget(target_pose2);
            std::cout<< "Grasping" << std::endl;
//            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
            move_group.setMaxVelocityScalingFactor(0.04);
            move_group.setMaxAccelerationScalingFactor(0.04);
            move_group.move();


            //闭合抓手
            ROS_INFO("closing the gripper .....");
            grasp_goal.width = 0.0;
            grasp_goal.speed = 0.05;
            grasp_goal.force = 1;
            grasp_goal.epsilon.inner = 0.2;
            grasp_goal.epsilon.outer = 0.2;
            ac_grasp.sendGoal(grasp_goal);
            SuccessGripper = ac_grasp.waitForResult(ros::Duration(10.0));
            std::cout << "gripper close Result: " << SuccessGripper << std::endl;



            //-----------------------------------------------------  move the end up  ------------------------------------------------------
            target_pose2.position.z = grasp_z ;  // 上升0.05米，保证夹抓移动路径不会碰到零件
            move_group.setPoseTarget(target_pose2);
//            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
            move_group.setMaxVelocityScalingFactor(0.3);
            move_group.setMaxAccelerationScalingFactor(0.3);
//            move_group.move();


            //-----------------------------------------------------  移动末端到放置零件的位置  ------------------------------------------------------

            ROS_INFO("Placing the objects...");
  joint_group_positions[0] = -0.004755951056354687;
  joint_group_positions[1] = -0.7816322353346304;
  joint_group_positions[2] = 0.0027716900604989447;
  joint_group_positions[3] = -2.357223240506345;
  joint_group_positions[4] = -0.00609153702422068;
  joint_group_positions[5] =  1.5533100152280594;
  joint_group_positions[6] =  0.788903862893995;

            move_group.setJointValueTarget(joint_group_positions);
            success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.setMaxVelocityScalingFactor(0.3);
            move_group.setMaxAccelerationScalingFactor(0.3);
//            move_group.move();


            //gripper open
            ROS_INFO("open the gripper .....");
            move_goal.width = 0.03;
            move_goal.speed = 0.1;
            ac_move.sendGoal(move_goal);
             //wait for the action to return
            SuccessGripper = ac_move.waitForResult(ros::Duration(10.0));
            std::cout << "gripper Result: " << SuccessGripper << std::endl;


            //-----------------------------------------------------  归位至等候位  ------------------------------------------------------

             ROS_INFO("Looking for grasp points .....");
  joint_group_positions[0] = -0.004755951056354687;
  joint_group_positions[1] = -0.7816322353346304;
  joint_group_positions[2] = 0.0027716900604989447;
  joint_group_positions[3] = -2.357223240506345;
  joint_group_positions[4] = -0.00609153702422068;
  joint_group_positions[5] =  1.5533100152280594;
  joint_group_positions[6] =  0.788903862893995;




             move_group.setJointValueTarget(joint_group_positions);
             success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
             move_group.setMaxVelocityScalingFactor(0.4);
             move_group.setMaxAccelerationScalingFactor(0.4);
//             move_group.move();


        }


        ros::spinOnce();
        loop_rate.sleep();
    }


  ros::waitForShutdown();
  return 0;
}

