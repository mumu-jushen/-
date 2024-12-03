#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <termio.h>
#include <stdio.h>
#include <sstream>
#include <tf/transform_broadcaster.h>



double pose_x;
double pose_y;
double pose_z;
double q_x;
double q_y;
double q_z;
double q_w;

///////////////////////////////////////  for Mpgrasp //////////////////////////////////////////////
//void object_pose_Callback(const std_msgs::Float64MultiArray &object_tmp)
//{
//    pose_x = object_tmp.data[0];
//    pose_y = object_tmp.data[1];
//    pose_z = object_tmp.data[2];
//    grasp_angle = object_tmp.data[3]; //rad
//    std::cout<<pose_x<< std::endl;
//}

///////////////////////////////////////  for graspnet //////////////////////////////////////////////
void object_pose_Callback(const std_msgs::Float64MultiArray &object_tmp)
{
    pose_x = object_tmp.data[0];
    pose_y = object_tmp.data[1];
    pose_z = object_tmp.data[2];


    q_x = object_tmp.data[3];
    q_y = object_tmp.data[4];
    q_z = object_tmp.data[5];
    q_w = object_tmp.data[6];
//    q_w = object_tmp.data[3];
//    q_x = object_tmp.data[4];
//    q_y = object_tmp.data[5];
//    q_z = object_tmp.data[6];
    std::cout<<pose_x<< std::endl;
    std::cout<<pose_y<< std::endl;
    std::cout<<pose_z<< std::endl;
    std::cout<<q_x<< std::endl;
    std::cout<<q_y<< std::endl;
    std::cout<<q_z<< std::endl;
    std::cout<<q_w<< std::endl;
}


int main (int argc, char** argv) {
    ros::init(argc, argv, "tf_tree");
    ros::NodeHandle nh;

    // read msg from grcnn
    ros::Subscriber sub = nh.subscribe("object_pose", 1000, object_pose_Callback);
    ros::Rate loop_rate(10);

    tf::TransformBroadcaster tf_camera_grasp;
    tf::TransformBroadcaster tf1;

    ROS_INFO("pub the tf tree");
    while (ros::ok())
    {

///////////////////////////////////////  for Mpgrasp //////////////////////////////////////////////
//        if (pose_z > 0.0)
//        {
//            tf_camera_grasp.sendTransform(
//                     tf::StampedTransform(
//                        tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(pose_x  , pose_y+0.005 , pose_z - 0.20 )   ),
//                        ros::Time::now(),"camera_link", "grasppoint"));
//         std::cout<<pose_x << ' ' << pose_y <<' ' << pose_z<< std::endl;
//        }
//        else
//        {
//            std::cout<<'no pose z'<< std::endl;
//        }

///////////////////////////////////////  for graspnet  //////////////////////////////////////////////


        tf_camera_grasp.sendTransform(
                 tf::StampedTransform(
                    tf::Transform(tf::Quaternion(q_x, q_y, q_z, q_w), tf::Vector3(pose_x  , pose_y , pose_z )   ),
                    ros::Time::now(),"camera_link", "graspgriper"));

        tf_camera_grasp.sendTransform(
                 tf::StampedTransform(
                    tf::Transform(tf::Quaternion(-0.000, -0.000, 0.383, 0.924), tf::Vector3(0.000, 0.000, -0.103 )   ),
                    ros::Time::now(),"graspgriper", "grasppoint"));

        tf1.sendTransform(
                tf::StampedTransform(
//                 tf::Transform(tf::Quaternion(0.018, 0.022, 0.702, 0.711), tf::Vector3(0.047, -0.020, -0.048)),
//                   tf::Transform(tf::Quaternion(0.023, -0.041, 0.731, 0.681), tf::Vector3(0.050, 0.007, -0.057)),
//                   ros::Time::now(),"panda_K", "camera_link"));
                   tf::Transform(tf::Quaternion(-0.005, -0.004, 0.396, 0.918), tf::Vector3(0.010, -0.062, 0.062)),
                   ros::Time::now(),"panda_link8", "camera_link"));


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
