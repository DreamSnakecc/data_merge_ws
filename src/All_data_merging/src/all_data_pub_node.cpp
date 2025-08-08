#include "All_data_merging/SerialData.h"
#include "All_data_merging/cmd.h"
#include "All_data_merging/AllData.h"
#include "NOKOV2ROS/robot_pose_vel.h"
#include "NOKOV2ROS/robot_pose_vel_accel.h"
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <fstream>
#include <iomanip>  // for std::setprecision
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "math.h"

using namespace std;

std::ofstream csv_file("/home/c/all_data_log.csv", std::ios::out | std::ios::app);  // 路径可改
bool csv_header_written = false;

// 使用 message_filters 进行时间同步
typedef message_filters::sync_policies::ApproximateTime<
    All_data_merging::SerialData, 
    All_data_merging::cmd,
    NOKOV2ROS::robot_pose_vel_accel       //NOKOV2ROS::robot_pose_vel
> SyncPolicy;

void writeDataToCSV(
                    const All_data_merging::SerialData::ConstPtr& serial_msg,
                    const All_data_merging::cmd::ConstPtr& cmd_msg,
                    const NOKOV2ROS::robot_pose_vel_accel::ConstPtr& pose_vel_accel_msg)
{
    if (!csv_file.is_open()) {
    ROS_ERROR("CSV file not open!");
    return;
    }

    if (!csv_header_written) {
        csv_file << "time,sec_f,cmd_value,"
                << "pose_x,pose_y,pose_z,roll,pitch,yaw,"
                << "linevel_x,linevel_y,linevel_z,angvel_x,angvel_y,angvel_z,"
                << "lineacc_x,lineacc_y,lineacc_z,angacc_x,angacc_y,angacc_z";

        // 添加所有串口字段名
        for (const auto& field : serial_msg->data_fields) {
            csv_file << "," << field.name;
        }
        csv_file << "\n";
        csv_header_written = true;
    }

    double time_sec = serial_msg->header.stamp.toSec();  // 时间戳使用serial为主
    csv_file << std::fixed << std::setprecision(6) << time_sec << ",";

    // 添加 sec_f 字段
    csv_file << std::setprecision(0) << pose_vel_accel_msg->sec_f << ",";

    // 添加 cmd 字段
    csv_file << cmd_msg->command << ",";

    Eigen::Quaternionf q4;
    // std::vector<float> pose_rpy;
    // pose_rpy.resize(3, 0.0);
    q4.setIdentity();
    // 更新四元数
    q4.x() = pose_vel_accel_msg->nokov_pose.pose.orientation.x;
    q4.y() = pose_vel_accel_msg->nokov_pose.pose.orientation.y;
    q4.z() = pose_vel_accel_msg->nokov_pose.pose.orientation.z;
    q4.w() = pose_vel_accel_msg->nokov_pose.pose.orientation.w;

    //计算欧拉角
    Eigen::Vector3f euler_angles = q4.toRotationMatrix().eulerAngles(0, 1, 2);
    // pose_rpy[0] = euler_angles[0]; // Roll
    // pose_rpy[1] = euler_angles[1]; // Pitch
    // pose_rpy[2] = euler_angles[2]; // Yaw

    // 添加位姿 速度和加速度
    csv_file << std::setprecision(6)
            << pose_vel_accel_msg->nokov_pose.pose.position.x << ","
            << pose_vel_accel_msg->nokov_pose.pose.position.y << ","
            << pose_vel_accel_msg->nokov_pose.pose.position.z << ","
            << euler_angles[0] << ","
            << euler_angles[1] << ","
            << euler_angles[2] << ","
            << pose_vel_accel_msg->nokov_vel.twist.linear.x << ","
            << pose_vel_accel_msg->nokov_vel.twist.linear.y << ","
            << pose_vel_accel_msg->nokov_vel.twist.linear.z << ","
            << pose_vel_accel_msg->nokov_vel.twist.angular.x << ","
            << pose_vel_accel_msg->nokov_vel.twist.angular.y << ","
            << pose_vel_accel_msg->nokov_vel.twist.angular.z << ","
            << pose_vel_accel_msg->nokov_accel.twist.linear.x << ","
            << pose_vel_accel_msg->nokov_accel.twist.linear.y << ","
            << pose_vel_accel_msg->nokov_accel.twist.linear.z << ","
            << pose_vel_accel_msg->nokov_accel.twist.angular.x << ","
            << pose_vel_accel_msg->nokov_accel.twist.angular.y << ","
            << pose_vel_accel_msg->nokov_accel.twist.angular.z ;

    // 添加所有串口字段值
    for (const auto& field : serial_msg->data_fields) {
        csv_file << "," << std::setprecision(6) << field.value;
    }
    csv_file << "\n";
     
}


// 声明全局发布器指针，将在main函数中初始化
ros::Publisher* all_data_pub_ptr = nullptr;

// 同步回调函数 - 当三个话题的数据时间戳对齐时调用
void SynchronizedCallback(
    const All_data_merging::SerialData::ConstPtr& serial_msg,
    const All_data_merging::cmd::ConstPtr& cmd_msg,
    const NOKOV2ROS::robot_pose_vel_accel::ConstPtr& pose_vel_msg)     //const NOKOV2ROS::robot_pose_vel::ConstPtr& pose_vel_msg
{            
    // 计算时间戳差异
    double max_time = std::max({
        serial_msg->header.stamp.toSec(),
        pose_vel_msg->header.stamp.toSec(),
        cmd_msg->header.stamp.toSec()
    });
    double min_time = std::min({
        serial_msg->header.stamp.toSec(),
        pose_vel_msg->header.stamp.toSec(),
        cmd_msg->header.stamp.toSec()
    });
    double time_diff = max_time - min_time;
    ROS_INFO("Time difference: %.6f seconds", time_diff);

    // 创建融合消息
    All_data_merging::AllData all_data_msg;
    all_data_msg.header = serial_msg->header;
    //all_data_msg.pose_vel = *pose_vel_msg;
    all_data_msg.pose_vel_accel = *pose_vel_msg;
    all_data_msg.serialdata = *serial_msg;
    all_data_msg.command = *cmd_msg;
    
    // 使用全局发布器指针发布消息
    if (all_data_pub_ptr) {
        all_data_pub_ptr->publish(all_data_msg);
    }

    writeDataToCSV(serial_msg,cmd_msg,pose_vel_msg);
    
}

int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "all_data_pub_node");

    //创建节点句柄
    ros::NodeHandle nh;
    
    // 创建所有ROS对象
    message_filters::Subscriber<All_data_merging::SerialData> serial_sub;
    message_filters::Subscriber<All_data_merging::cmd> cmd_sub;
    //message_filters::Subscriber<NOKOV2ROS::robot_pose_vel> pose_vel_sub;   
    message_filters::Subscriber<NOKOV2ROS::robot_pose_vel_accel> pose_vel_sub;

    // 初始化 message_filters 订阅器
    serial_sub.subscribe(nh, "/serial_data", 3);
    //pose_vel_sub.subscribe(nh, "/pose_vel_data", 3);
    pose_vel_sub.subscribe(nh, "/pose_vel_accel_data", 3);
    cmd_sub.subscribe(nh, "/cmd_data", 3);

    // 创建发布器
    ros::Publisher all_data_pub = nh.advertise<All_data_merging::AllData>("/all_data", 1);
    all_data_pub_ptr = &all_data_pub;  // 设置全局指针

    // 设置同步策略
    // 参数: 队列大小为3，最大时间差为0.02秒
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer;
    synchronizer.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(3), 
                                                              serial_sub, 
                                                              cmd_sub, 
                                                              pose_vel_sub));
    synchronizer->setMaxIntervalDuration(ros::Duration(0.02));

    // 注册同步回调函数
    synchronizer->registerCallback(boost::bind(&SynchronizedCallback, _1, _2, _3));

    ROS_INFO("all_data_pub_node started, waiting for synchronized messages...");
    
    ros::spin();
    csv_file.close();
    ROS_INFO("CSV file closed.");
    return 0;
}
