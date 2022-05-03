#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Int32.h"
//以下为串口通讯需要的头文件
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::copy;
// using std::begin;
// using std::end;
/*****************************************************************************/
float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
float D = 0.5f ;    //两轮间距，单位是m
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
/****************************************************/
unsigned char data_terminal0=0x28;  //“("字符
unsigned char data_terminal1=0x29;  //“)"字符
unsigned char speed_data[12];   //要发给串口的数据
unsigned char speed_data_copy[12];
string rec_buffer;  //串口数据接收变量
serial::Serial my_serial;
//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular;

// "^" function
unsigned char exclusiveOr(unsigned char arr[],int beginIndex)
{
    for(int i = beginIndex;i<beginIndex+7;i++)
    {
        arr[i+1]=arr[i]^arr[i+1];
    }
    return arr[beginIndex+8];
}

int  moveDirection(float leftSpeed,float rightSpeed)
{
    if (leftSpeed<0.0&&rightSpeed<0.0)
    {
        return 0x61;
    }
   if (leftSpeed<0.0&&rightSpeed>0.0)
    {
        return 0x62;
    }
        if (leftSpeed>0.0&&rightSpeed<0.0)
    {
        return 0x63;
    }
   if (leftSpeed>0.0&&rightSpeed>0.0)
    {
        return 0x64;
    }
	return 0;
}

void dataprint(unsigned char arr[])
{
    for(int i=2 ; i<10 ; i++)
    {
       cout<<(float)arr[i];
    }
    cout<<endl;
}
void modecallback(const std_msgs::Int32::ConstPtr& msg){
	string port("/dev/ttyUSB0");//小车串口号
    unsigned long baud = 115200;//小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口
	speed_data[0]=data_terminal0;
	for(int i = 0 ; i < 10 ; ++i){
		speed_data[i+1] = 0x65;
	}
	speed_data[11]=data_terminal1;
	if(msg->data == 1){my_serial.write(speed_data,12);}
	cout<<"model"<<endl;
}
/*********************订阅/cmd_vel主题回调函数***************************************/
void callback(const geometry_msgs::Twist & cmd_input)
{
    string port("/dev/ttyUSB0");//小车串口号
    unsigned long baud = 115200;//小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口

    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s

    //将转换好的小车速度分量为左右轮速度
    left_speed_data.d = linear_temp - 0.5f*angular_temp*D ;
    right_speed_data.d = linear_temp + 0.5f*angular_temp*D ;

    //存入数据到要发布的左右轮速度消息
    left_speed_data.d*=ratio;   //放大１０００倍，mm/s
    right_speed_data.d*=ratio;//放大１０００倍，mm/s
    speed_data[0]=data_terminal0;
    speed_data[1]=moveDirection(left_speed_data.d,right_speed_data.d);//robot move direction

    // cout<<"leftSpeed:"<<left_speed_data.d<<endl;
    // dataprint(left_speed_data.data);
    // cout<<"rightSpeed:"<<right_speed_data.d<<endl;
    // dataprint(right_speed_data.data);


    for(int i=2;i<6;i++)    //将左右轮速度存入数组中发送给串口
    {
        speed_data[i]=right_speed_data.data[i];
        speed_data[i+4]=left_speed_data.data[i];
        // speed_data[i] = '1';
    }

    //在写入串口的左右轮速度数据加入”()“

    // copy(begin(speed_data),end(speed_data),begin(speed_data_copy));
    memcpy(speed_data_copy,speed_data,12);
    // speed_data[10]=0x48;
    speed_data[10] = exclusiveOr(speed_data_copy,2);
    speed_data[11]=data_terminal1;
    //写入数据到串口
    my_serial.write(speed_data,12);
    dataprint(speed_data);
    cout << "success!!" << endl;
    
}


int main(int argc, char **argv)
{
    string port("/dev/ttyUSB0");//小车串口号
    unsigned long baud = 115200;//小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口
    
    ros::init(argc, argv, "wheel_control_v1");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
	ros::Subscriber modesub = n.subscribe("model_change", 1000, modecallback); //订阅/cmd_vel主题
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback); //订阅/cmd_vel主题
    
    ros::Rate loop_rate(10);//设置周期休眠时间
    ros::spin();
    return 0;
}
