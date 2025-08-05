#include "All_data_merging/serial_debug.h"
#include "All_data_merging/SerialData.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <cstring>
#include <iostream>
#include <yaml-cpp/yaml.h>

using namespace std;

//串口接收定时器
ros::Timer serial_data_timer;
//串口对象
Serial_Debug serial_debug;
//串口发布器 (原来的固定结构消息)
ros::Publisher serial_data_pub;


//rosbag record -O test /serial_data
int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "serial_debug_node");
    if (argc != 2)
    {
        ROS_INFO("Serial_Debug:Please set 1 serial port!");
        return 1;
    }

    //创建节点句柄
    ros::NodeHandle nh;

    //初始化串口
    serial_debug.SerialInit(argv[1]);

    //初始化发布器
    serial_data_pub = nh.advertise<All_data_merging::SerialData>("/serial_data",1);

    //command串口读取定时器(周期为5ms),创建一个线程执行耗时操作
    std::thread worker(SerialData_parser);
    serial_data_timer = nh.createTimer(ros::Duration(0.005), SerialDataCallback);

    ros::spin();
    
    return 0;
}

