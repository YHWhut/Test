#include "ros/ros.h"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

#include "data_generator.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

#define ROW 480
#define COL 752


int main(int argc, char** argv)
{
    // ros::init(argc, argv, "move_base_simple"); 
    ros::init(argc, argv, "data_generator");

    ros::NodeHandle n("~");

    ros::Publisher pub_path     = n.advertise<nav_msgs::Path>("path", 1000);
    ros::Publisher pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    // ros::Publisher pub_pose     = n.advertise<geometry_msgs::PoseStamped>("goal", 1000);
    ros::Publisher pub_pose_    = n.advertise<geometry_msgs::Point>("path_point",1000);

    ros::Duration(1).sleep();

    DataGenerator generator;
    ros::Rate loop_rate(generator.FREQ);

    int publish_count = 0;

    //发布轨迹
    nav_msgs::Path path;
    path.header.frame_id = "odom";	
    Vector3d pre_p;
    double fst_del_d = -1;
    while (ros::ok())
    {
        double current_time = generator.getTime();
        ROS_INFO("time: %lf", current_time);
        Vector3d position     = generator.getPosition();
        Vector3d velocity     = generator.getVelocity();
        // Matrix3d rotation     = generator.getRotation();
        // Quaterniond q(rotation);
        Quaterniond q = generator._getRotation();

        Vector3d linear_acceleration = generator.getLinearAcceleration();
        Vector3d angular_velocity = generator.getAngularVelocity();

        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "odom";
        odometry.header.stamp = ros::Time(current_time);

        odometry.pose.pose.position.x = position(0);
        odometry.pose.pose.position.y = position(1);
        odometry.pose.pose.position.z = position(2);
        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.pose.pose.orientation.w = q.w();
        
        odometry.twist.twist.linear.x = velocity(0);
        odometry.twist.twist.linear.y = velocity(1);
        odometry.twist.twist.linear.z = velocity(2);
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.header.stamp = ros::Time(current_time);
        pose_stamped.pose = odometry.pose.pose;
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);
        // pub_pose.publish(pose_stamped);

        geometry_msgs::Pose point;
        point.position.x = odometry.pose.pose.position.x;
        point.position.y = odometry.pose.pose.position.y;
        point.position.z = 0.0;
        pub_pose_.publish(point);
        //update work
        generator.update();
        publish_count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
