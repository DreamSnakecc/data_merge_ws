#include "NOKOV2ROS/get_pose_vel.h"
#include "NOKOV2ROS/robot_pose_vel.h"
#include "NOKOV2ROS/robot_pose_vel_accel.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
// ===== GET_POSE_VEL 类实现 =====
typedef message_filters::sync_policies::ApproximateTime<
    geometry_msgs::PoseStamped,
    geometry_msgs::TwistStamped,
    geometry_msgs::TwistStamped> SyncPolicy;
    
GET_POSE_VEL::GET_POSE_VEL(ros::NodeHandle &nh, std::string climbot_name, const float &frequency, AlgorithmType algorithm_type)
    : nh_(nh), climbot_name_(climbot_name), frequency_(frequency)
{
       
    //vrpn发布的话题名称格式，climbot_name是XINGYING中定义的刚体名称
    std::string pose_topic_name = "vrpn_client_ros/" + climbot_name + "/pose";
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, pose_topic_name, 3);       
    
    std::string twist_topic_name = "vrpn_client_ros/" + climbot_name + "/twist";
    message_filters::Subscriber<geometry_msgs::TwistStamped> vel_sub(nh, twist_topic_name, 3);

    std::string accel_topic_name = "vrpn_client_ros/" + climbot_name + "/accel";
    message_filters::Subscriber<geometry_msgs::TwistStamped> accel_sub(nh, accel_topic_name, 3);

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
    sync.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(3), pose_sub, vel_sub, accel_sub));
    sync->setMaxIntervalDuration(ros::Duration(0.02));
    sync->registerCallback(boost::bind(&GET_POSE_VEL::NokovCallback, this,_1, _2, _3));

    ROS_INFO("pose_vel_acc_sync_node started.");
    // 初始化发布器
    pose_vel_accel_pub_ = nh_.advertise<NOKOV2ROS::robot_pose_vel>("/pose_vel_accel_data", 1);
}


void GET_POSE_VEL::NokovCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg,
                                 const geometry_msgs::TwistStampedConstPtr& vel_msg,
                                 const geometry_msgs::TwistStampedConstPtr& accel_msg)
{

    // 计算同步误差
    double max_t = std::max({pose_msg->header.stamp.toSec(), vel_msg->header.stamp.toSec(), accel_msg->header.stamp.toSec()});
    double min_t = std::min({pose_msg->header.stamp.toSec(), vel_msg->header.stamp.toSec(), accel_msg->header.stamp.toSec()});
    double diff = max_t - min_t;
    if (diff > 0.02) {
        ROS_WARN("Sync error: %.6f seconds. Skipping message.", diff);
        return;
    }

    // 组装自定义消息
    NOKOV2ROS::robot_pose_vel_accel fused_msg;
    fused_msg.header.stamp = pose_msg->header.stamp;
    fused_msg.header.frame_id = pose_msg->header.frame_id;

    fused_msg.nokov_pose = *pose_msg;
    fused_msg.nokov_vel = *vel_msg;
    fused_msg.nokov_accel = *accel_msg;

    fused_msg.sec_f = (double)(pose_msg->header.stamp.nsec)/1e9 + pose_msg->header.stamp.sec;

    pose_vel_accel_pub_.publish(fused_msg);

    
}


