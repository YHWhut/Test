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
#include "string.h"
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::copy;

/*****************************************************************************/
int ratio = 1000 ;   //转速转换比例，执行速度调整比例
float D = 0.5f ;    //两轮间距，单位是m
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
float linear_y,linear_z;
/****************************************************/
unsigned char data_terminal0=0x28;  //“("字符
unsigned char data_terminal1=0x29;  //“)"字符
unsigned char speed_data[12];   //要发给串口的数据
unsigned char contral_data[3];

string rec_buffer;  //串口数据接收变量

//发送给下位机的左右轮速度，里程计的坐标和方向
unsigned char right_speed_data[4],left_speed_data[4];

/*
函数功能：计算八位循环冗余校验
入口参数：数组地址、数组大小
*/
unsigned char exclusiveOr(unsigned char* ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while (len--)
    {
        crc ^= *ptr++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// int  moveDirection(int leftSpeed,int rightSpeed)
// {
//     if (leftSpeed<0.0&&rightSpeed<0.0)
//     {
//         return 0x61;
//     }
//    if (leftSpeed<0.0&&rightSpeed>0.0)
//     {
//         return 0x62;
//     }
//         if (leftSpeed>0.0&&rightSpeed<0.0)
//     {
//         return 0x63;
//     }
//    if (leftSpeed>0.0&&rightSpeed>0.0)
//     {
//         return 0x64;
//     }
//     else
//         return 0;
// }

unsigned char* dataUpdate(unsigned char ch[])
{
    for(int i=4;i<6;i++)
    {
        switch (ch[i])
        {
        case 0x28:
            ch[i] = 0x27;
            break;
        case 0x29:
            ch[i] = 0x30;
            break;
        default:
            break;
        }
        switch (ch[i+4])
        {
        case 0x28:
            ch[i+4] = 0x27;
            break;
        case 0x29:
            ch[i+4] = 0x30;
            break;
        default:
            break;
        }
    }
    return ch;
}

/*********************订阅/cmd_vel主题回调函数***************************************/
void callback(const geometry_msgs::Twist & cmd_input)
{
    string port("/dev/ttyUSB0");    //小车串口号
    unsigned long baud = 9600;    //小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口

    geometry_msgs::Twist cmd_input_ = cmd_input;
    angular_temp = cmd_input_.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input_.linear.x ;//获取/cmd_vel的线速度.m/s
    linear_y = cmd_input_.linear.y;
    linear_z = cmd_input_.linear.z;
    if(linear_y == 0 && linear_z == 0)
    {
        //将转换好的小车速度分量为左右轮速度
        float left_speed = (linear_temp - 0.5f*angular_temp*D) ;
        float right_speed = (linear_temp + 0.5f*angular_temp*D) ;

        //存入数据到要发布的左右轮速度消息
        int leftt = (int)(left_speed*ratio);   //放大１０００倍，mm/s
        int rightt = (int)(right_speed*ratio);//放大１０００倍，mm/s
        int diff = abs(leftt - rightt) / 2;
        double gain = 0.0;
        leftt > rightt ? leftt+gain*diff : rightt+gain*diff;
        leftt > rightt ? rightt-gain*diff : leftt-gain*diff;
        if(rightt >= 0)
        {
            right_speed_data[1]=0;
            right_speed_data[0]=0;
        }
        else
        {
            right_speed_data[1]=0xff;
            right_speed_data[0]=0xff;
        }
        right_speed_data[2]=abs(rightt)/256;
        right_speed_data[3]=abs(rightt)%256;   
        if(leftt >= 0)
        {
            left_speed_data[0]=0;
            left_speed_data[1]=0;
        }
        else
        {
            left_speed_data[0]=0xff;
            left_speed_data[1]=0xff;
        }
        left_speed_data[2]=abs(leftt)/256; 
        left_speed_data[3]=abs(leftt)%256; 
        
        speed_data[0]=data_terminal0;
        // speed_data[1]=moveDirection(leftt,rightt);//robot move direction
        speed_data[1]=0x00;//robot move direction



        for(int i=2;i<6;i++)    //将左右轮速度存入数组中发送给串口
        {
            speed_data[i]=right_speed_data[i-2];
            speed_data[i+4]=left_speed_data[i-2];
            printf("-----%02x ",right_speed_data[i-2]);
        }

        //在写入串口的左右轮速度数据加入”()“
        speed_data[10]=exclusiveOr(dataUpdate(speed_data),10);
        speed_data[11]=data_terminal1;
        //写入数据到串口
        if(speed_data[10] != 0x28 && speed_data[10] != 0x29)
        {
            my_serial.write(dataUpdate(speed_data),12);
        }
        printf("%x %x 右：%x %x %x %x 左：%x %x %x %x 校验：%x  %x\r\n",speed_data[0],speed_data[1],speed_data[2],speed_data[3],speed_data[4],speed_data[5],\
        speed_data[6],speed_data[7],speed_data[8],speed_data[9],speed_data[10],speed_data[11]);
        cout<<"右轮速度："<<rightt<<" "<<"左轮速度："<<leftt<<endl;

    }
    else if(linear_y != 0.0)
    {
        contral_data[0] = 0x28;
        contral_data[1] = 0x61;
        contral_data[2] = 0x29;
        my_serial.write(contral_data,3);
        printf("%x\n",contral_data[1]);
    }
    else if(linear_z != 0.0)
    {
        contral_data[0] = 0x28;
        contral_data[1] = 0x73;
        contral_data[2] = 0x29;
        my_serial.write(contral_data,3);
        printf("%x\n",contral_data[1]);
    }

}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    // string port("/dev/ttyUSB0");//小车串口号
    // unsigned long baud = 115200;//小车串口波特率
    // serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口
    
    ros::init(argc, argv, "wheel_control_v1");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback); //订阅/cmd_vel主题
	// ros::Subscriber modesub = n.subscribe("model_change", 1000, modecallback); //订阅/cmd_vel主题
    
    ros::Rate loop_rate(10);//设置周期休眠时间
    ros::spin();
    return 0;
}
