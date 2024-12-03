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
double grasp_angle;

void object_pose_Callback(const std_msgs::Float64MultiArray &object_tmp)
{
    pose_x = object_tmp.data[0];
    pose_y = object_tmp.data[1];
    pose_z = object_tmp.data[2];
    grasp_angle = object_tmp.data[3]; //rad
    std::cout<<pose_x<< std::endl;
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
        // observation pose
        if (pose_z > 0.0)
        {
                tf_camera_grasp.sendTransform(
                tf::StampedTransform(
//                        tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(pose_x +0.017 , pose_y - 0.003, pose_z + 0.0012)),
                        tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(pose_x  , pose_y , pose_z )),
                        ros::Time::now(),"camera_color_frame", "grasppoint"));
         std::cout<<pose_x << ' ' << pose_y <<' ' << pose_z<< std::endl;
        }

        tf1.sendTransform(
        tf::StampedTransform(
                        tf::Transform(tf::Quaternion(-0.003, -0.0005, 0.717, 0.697), tf::Vector3(0.051, -0.037 , -0.042)),
                        ros::Time::now(),"panda_K", "camera_color_frame"));





        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
