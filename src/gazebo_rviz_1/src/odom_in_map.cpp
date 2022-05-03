#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/OccupancyGrid.h"
//#include "tf2/utils.h"
#include "math.h"
//#include "tf2/convert.h"
//#include "tf2_ros/message_filter.h"
using namespace std;
 
tf::TransformListener *tf_;
tf::TransformBroadcaster *tfb_;
nav_msgs::Odometry::ConstPtr oppp;
 
// The basic vector
typedef struct
{
  double v[3]={0};
} pose_vector_t;
 
static double normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}
 
// Return a zero vector
pose_vector_t pose_vector_zero()
{
  pose_vector_t c;
 
  c.v[0] = 0.0;
  c.v[1] = 0.0;
  c.v[2] = 0.0;
 
  return c;
}
 
void pose_vector_setValue(pose_vector_t * c,double x,double y,double yaw)
{
    double *v;
    v=c->v;
    *v=x;
    v++;
    *v=y;
    v++;
    *v=yaw;
    //c->(v+1)=y;
    //*c->(v=yaw;
}
 
pose_vector_t lastPose_v;
tf::Transform lastTransfrom_map_in_odom;
 
void odomMsgCallback(const nav_msgs::Odometry::ConstPtr &odomMsg){
 
    static bool init=false;
    // static double delta_x=0;
    static bool forward=true;
    ros::Duration transform_tolerance_;
    transform_tolerance_.fromSec(0.1);
    //位姿偏移量初始化
    pose_vector_t delta = pose_vector_zero();
    lastTransfrom_map_in_odom=tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),
            tf::Vector3(0, 0, 0));
 
    if(!init){
        init=true;
        pose_vector_setValue(&lastPose_v,
                             odomMsg->pose.pose.position.x,
                             odomMsg->pose.pose.position.y,
                             tf::getYaw(odomMsg->pose.pose.orientation));
 
    }else{
        delta.v[0] = odomMsg->pose.pose.position.x - lastPose_v.v[0];
        delta.v[1] = odomMsg->pose.pose.position.y - lastPose_v.v[1];
        delta.v[2] = angle_diff(tf::getYaw(odomMsg->pose.pose.orientation), lastPose_v.v[2]);
 
        //判断位移偏移量是否大于阈值
        if(true/*sqrt(pow(delta.v[0],2)+pow(delta.v[1],2))>=0*/){
 
            /******************发布坐标变换**********************************/
            //取odom获取的位姿作为真实位姿
            pose_vector_t truePose_v;
            truePose_v.v[0]=odomMsg->pose.pose.position.x;
            truePose_v.v[1]=odomMsg->pose.pose.position.y;
            truePose_v.v[2]=tf::getYaw(odomMsg->pose.pose.orientation);
 
            tf::Stamped<tf::Transform> map_in_odom;
            tf::Stamped<tf::Transform> odom_in_map;
            //tf::Transform map_in_odom;
            try{
                //创建一个基于global map的坐标
                tf::Stamped<tf::Pose> ident(tf::Transform(tf::createQuaternionFromRPY(0, 0, truePose_v.v[2]),
                                                          tf::Vector3(truePose_v.v[0], truePose_v.v[1], 0)),odomMsg->header.stamp, "map");
                //创建上面坐标的逆，即global map原点在Base_link坐标系下的坐标
                tf::Stamped<tf::Pose> tmp(ident.inverse(),odomMsg->header.stamp, "base_link");
                //然后将该坐标转换到odom坐标系下
                //得到的map_in_odom是 global map原点在odom坐标系下的坐标
                tf_->transformPose("odom", tmp, map_in_odom);
 
                //求逆变换
                //得到的odom_in_map是 odom坐标原点在global map坐标系下的坐标
                //即得到从odom坐标系到global map坐标系的变换矩阵
                odom_in_map.setData(map_in_odom.inverse());
                odom_in_map.frame_id_="map";
                odom_in_map.stamp_=odomMsg->header.stamp+transform_tolerance_;
 
                ROS_INFO("calculate odom in map success");
                ROS_INFO("odom in map x:[%f] y:[%f] yaw:[%f]",
                         odom_in_map.getOrigin().x(),
                         odom_in_map.getOrigin().y(),
                         odom_in_map.getRotation().getAngle());
 
                ROS_INFO("now send the TF Broadcast odom_in_map");
 
                tfb_->sendTransform(tf::StampedTransform(odom_in_map,odom_in_map.stamp_,"map","odom"));
 
                lastTransfrom_map_in_odom=odom_in_map;
 
            }catch(tf::TransformException e){
                ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
                return ;
            }
//            ROS_INFO("now send the TF Broadcast odom_in_map");
//            odom_in_map.setData(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),
//                                                          tf::Vector3(0, 0, 0)));
//            odom_in_map.frame_id_="map";
//            odom_in_map.stamp_=odomMsg->header.stamp+transform_tolerance_;
//            tfb_->sendTransform(tf::StampedTransform(odom_in_map,odom_in_map.stamp_,"map","odom"));
            // if(delta_x<=0){
            //     forward=true;
            // }
            // if(delta_x>=10){
            //     forward=false;
            // }
            // if(forward)
            //     delta_x+=0.02;
            // else
            //     delta_x-=0.02;
 
//            pose_vector_setValue(&lastPose_v,
//                                 odomMsg->pose.pose.position.x,
//                                 odomMsg->pose.pose.position.y,
//                                 tf::getYaw(odomMsg->pose.pose.orientation));
        }else{
            //位移偏移量没有达到阈值
            //发布之前的变换
            ROS_INFO("now send the TF Broadcast odom_in_map");
            tfb_->sendTransform(tf::StampedTransform(lastTransfrom_map_in_odom,odomMsg->header.stamp+transform_tolerance_,"map","odom"));
        }
    }
 
//    tf::Stamped<tf::Transform> odom_in_map;
//    tfb_->sendTransform(tf::StampedTransform(odom_in_map,odom_in_map.stamp_,"map","odom"));
 
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_in_map");
    ros::NodeHandle nh;
 
    tf_=new tf::TransformListener;
    tfb_=new tf::TransformBroadcaster;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_;
    //message_filters::Subscriber<sensor_msgs::LaserScan>* scan_sub_;
    tf::MessageFilter<nav_msgs::Odometry>* odom_filter_;
 
    // Subscribers
    //订阅"odom"
    ROS_INFO("subscribed the topic \"odom\" ");
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "odom", 100);
    //odom_sub_->registerCallback(odomMsgCallback/*boost::bind(odomMsgCallback, this, _1)*/);
    odom_filter_ =new tf::MessageFilter<nav_msgs::Odometry>(*odom_sub_, *tf_, "base_link", 100);
    odom_filter_->registerCallback(odomMsgCallback);
    ros::spin();
 
}