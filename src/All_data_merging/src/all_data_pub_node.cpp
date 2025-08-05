#include "All_data_merging/SerialData.h"
#include "All_data_merging/cmd.h"
#include "All_data_merging/AllData.h"
#include "NOKOV2ROS/robot_pose_vel.h"
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>

using namespace std;

// 使用 message_filters 进行时间同步
typedef message_filters::sync_policies::ApproximateTime<
    All_data_merging::SerialData, 
    All_data_merging::cmd,
    NOKOV2ROS::robot_pose_vel
> SyncPolicy;

// 声明全局发布器指针，将在main函数中初始化
ros::Publisher* all_data_pub_ptr = nullptr;

// 同步回调函数 - 当三个话题的数据时间戳对齐时调用
void SynchronizedCallback(
    const All_data_merging::SerialData::ConstPtr& serial_msg,
    const All_data_merging::cmd::ConstPtr& cmd_msg,
    const NOKOV2ROS::robot_pose_vel::ConstPtr& pose_vel_msg)
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
    all_data_msg.pose_vel = *pose_vel_msg;
    all_data_msg.serialdata = *serial_msg;
    all_data_msg.command = *cmd_msg;
    
    // 使用全局发布器指针发布消息
    if (all_data_pub_ptr) {
        all_data_pub_ptr->publish(all_data_msg);
    }
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
    message_filters::Subscriber<NOKOV2ROS::robot_pose_vel> pose_vel_sub;
    
    // 初始化 message_filters 订阅器
    serial_sub.subscribe(nh, "/serial_data", 3);
    pose_vel_sub.subscribe(nh, "/pose_vel_data", 3);
    cmd_sub.subscribe(nh, "/cmd_data", 3);

    // 创建发布器
    ros::Publisher all_data_pub = nh.advertise<All_data_merging::AllData>("/all_data", 1);
    all_data_pub_ptr = &all_data_pub;  // 设置全局指针

    // 设置同步策略
    // 参数: 队列大小为3，最大时间差为0.01秒
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer;
    synchronizer.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(3), 
                                                              serial_sub, 
                                                              cmd_sub, 
                                                              pose_vel_sub));
    synchronizer->setMaxIntervalDuration(ros::Duration(0.01));

    // 注册同步回调函数
    synchronizer->registerCallback(boost::bind(&SynchronizedCallback, _1, _2, _3));

    ROS_INFO("all_data_pub_node started, waiting for synchronized messages...");
    
    ros::spin();
    
    return 0;
}
