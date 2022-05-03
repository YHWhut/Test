#pragma once
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
// #include "spinner.h"
// #include "sensor_msgs/Imu.h"
// #include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <vector>
#include <cstdlib>
#include <pthread.h>
using namespace std;
class CmdVel
{
private:
    ros::Subscriber imu_sub, path_point_sub;
    double angular_velocity;
    // double x_acc;
    // double y_acc;
    double pi = 3.14159;
    Eigen::Vector3d euler;
    Eigen::Quaterniond q;
    geometry_msgs::Point point;
    double oriz_w,oriz_z;
public:
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_msgs;
    vector<geometry_msgs::Point> v_points;
    // vector<vector<geometry_msgs::Point> *> v_allpoints;
    // vector<geometry_msgs::Point> points_1;
    // vector<geometry_msgs::Point> points_2;
    // vector<geometry_msgs::Point> points_3;
    // vector<geometry_msgs::Point> points_4;
    vector<double> v_yaw_output;

    CmdVel()
    {
        imu_sub = nh.subscribe("/odom", 1, &CmdVel::odom_callback,this);
        // path_point_sub = nh.subscribe("/tr_node/path_point", 1, &CmdVel::path_point_callback,this);
        path_point_sub = nh.subscribe("/data_generator/path_point", 1, &CmdVel::path_point_callback,this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 2);
        nh.param("linear_Kp",this->Kp2,0.0);
        nh.param("linear_Ki",this->Ki2,0.0);
        nh.param("linear_Kd",this->Kd2,0.0);
        nh.param("angular_Kp",this->Kp,0.0);
        nh.param("angular_Ki",this->Ki,0.0);
        nh.param("angular_Kd",this->Kd,0.0);
        this->isDown = false;
        this->isDown_ = false;

    }
    ~CmdVel()
    {

    }
    //起始点
    double x_current = 0.0;
    double y_current = 0.0;
    double yaw_current = 0.0;
    //目标点初始化
    double x_goal = 0.0;
    double y_goal = 0.0;
    double T = 0.1;//odom发布频率的倒数
    double L =  0.5;//轮距
    //PID控制参数初始化
    //yaw角
    double Kp;
    double Ki;
    double Kd;

    //线速度
    double Kp2;
    double Ki2;
    double Kd2;


    double yaw_out;
    double v_out;
    bool isDown;
    bool isDown_;
    bool AllPointsStoraged(const vector<geometry_msgs::Point> &points);
    void YawOutputCalculation(vector<geometry_msgs::Point> &points);
    double VelocityOutputCalculation(const vector<geometry_msgs::Point> &points, const int &index1, const int &index2);
    void path_point_callback(const geometry_msgs::Point::ConstPtr &point_msg)
    {
        ros::Rate r(10);
        point.x = point_msg->x;
        point.y = point_msg->y;
        point.z = 0.0;
        if(!AllPointsStoraged(v_points))
        {
            v_points.push_back(point);
        }
        r.sleep();
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        if(isDown)
        {

            q.x() = odom_msg->pose.pose.orientation.x;
            q.y() = odom_msg->pose.pose.orientation.y;
            q.z() = oriz_z = odom_msg->pose.pose.orientation.z;
            q.w() = oriz_w = odom_msg->pose.pose.orientation.w;

            euler = q.toRotationMatrix().eulerAngles(2,1,0);
            if(oriz_w > 0)
            {
                if(oriz_z < 0)
                {
                    yaw_current = euler[0] + pi;
                }
                else{yaw_current = euler[0];}
            }
            else
            {
                if(oriz_z < 0)
                {
                    yaw_current = euler[0];
                }
                else{yaw_current = euler[0] + pi;}
            }
            angular_velocity = odom_msg->twist.twist.angular.z;

            x_current =odom_msg->pose.pose.position.x;
            y_current =odom_msg->pose.pose.position.y;
            while(v_out > 1.0)
            {
                v_out -= 0.1;
            }
            cmd_msgs.angular.z = yaw_out;
            cmd_msgs.linear.x = v_out;
            cmd_msgs.linear.y = 0.0;
            cmd_vel_pub.publish(cmd_msgs);

        }

    }



};