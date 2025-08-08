/*
 * @Author: lxs
 * @Date: 2022-11-29 03:14:39
 * @LastEditTime: 2024-06-12 08:24:10
 * @LastEditors: lxs
 * @Description: 
 * @FilePath: /double_steer/src/steer_track/include/steer_track/serial_debug_node.h
 * Copyright (c) 2021 LXScience&Technology. All rights reserved.
 */
#ifndef CLIMBOT_VEL_CONTROL_SERIAL_DEBUG
#define CLIMBOT_VEL_CONTROL_SERIAL_DEBUG

#include <ros/ros.h>
#include <serial/serial.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <queue>
#include <ctime>
#include <mutex>
#include <algorithm>
#include <condition_variable>
#include <iostream>
#include "All_data_merging/SerialData.h"

class Serial_Debug
{
private:
    serial::Serial ros_ser_;  // 添加成员变量
    
public:
    /*! 
    * \brief 初始化串口
    * 
    * \param port Jetson nano串口的端口
    */
    void SerialInit(std::string port);

    /*!
    * \brief 关闭串口
    */
    void CloseSerial();

    /*!
   * \brief 读取串口数据
   * 
   * \return 串口缓冲区数据
   */
    std::string SerialRead();

    /*** 
     * @brief: 
     * @param {const} std
     * @return {*}
     */    
    size_t SerialWrite(const std::string str);

    void float2hex(float f, char h[4]);
    void double2hex(double d, char h[8]);

};

void SerialDataCallback(const ros::TimerEvent&);
void SerialData_parser();

#endif