#ifndef CLIMBOT_VEL_CONTROL_GET_POSE_VEL
#define CLIMBOT_VEL_CONTROL_GET_POSE_VEL

#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "math.h"
#include "NOKOV2ROS/robot_pose_vel.h"

// 前向声明
class DifferentialAlgorithm;

class GET_POSE_VEL
{
protected:
    ros::NodeHandle nh_;
    std::string climbot_name_;          //机器人名称（XINGYING软件定义刚体名称）
    std::vector<float> pose_;           //机器人位姿
    Eigen::Quaternionf q4_;             //机器人位姿四元数
    std::vector<float> curr_vel_;       //机器人速度
    std::vector<float> curr_accel_;     //机器人加速度
    std::vector<std::vector<float>> pose_queue_;          //位姿序列
    size_t pose_queue_size_;          //位姿队列大小
    std::vector<std::vector<float>> curr_vel_queue_;      //速度序列
    ros::Subscriber pose_sub_ ;     //机器人位姿订阅器
    ros::Subscriber vel_sub_ ;      //机器人速度订阅器
    ros::Subscriber accel_sub_ ;     //机器人加速度订阅器
    ros::Publisher pose_vel_pub_;   //位姿和速度发布器
    ros::Publisher pose_vel_accel_pub_;   //位姿/速度/加速度发布器
    float frequency_;             //机器人位姿采样频率
    DifferentialAlgorithm *diff_algorithm_;// 差分算法指针
    
    const float fir_a_[26] = {
  -0.0006956761794133,-0.002120592795595,-0.004378002076251,-0.006705587509768,
    -0.0073969593179,-0.003939494651136, 0.006321671791697,   0.0251365935248,
    0.05228012542714,  0.08494130810849,   0.1179562851727,   0.1449676030656,
     0.1601932777121,   0.1601932777121,   0.1449676030656,   0.1179562851727,
    0.08494130810849,  0.05228012542714,   0.0251365935248, 0.006321671791697,
  -0.003939494651136,  -0.0073969593179,-0.006705587509768,-0.004378002076251,
  -0.002120592795595,-0.0006956761794133
};                //FIR滤波器系数

public:
    /*!
    * \brief 算法类型枚举
    */
    enum AlgorithmType {
        FIRST_ORDER_DIFF = 1,    // 一阶差分
        SECOND_ORDER_BWD_DIFF = 2 // 二阶向后差分
    };

    /*!
    * \brief 构造函数
    * 
    * \param nh ROS节点句柄
    * \param climbot_name Motive定义的机器人名称
    * \param frequency 采样频率
    * \param algorithm_type 差分算法类型
    */
    GET_POSE_VEL(ros::NodeHandle &nh, std::string climbot_name, const float &frequency, AlgorithmType algorithm_type = FIRST_ORDER_DIFF);
    //GET_POSE_VEL(ros::NodeHandle &nh, std::string climbot_name, const float &frequency);
    /*!
    * \brief 计算速度（使用指定的差分算法）
    * 
    */
    void CalculateVel();

    /*!
    * \brief 位姿订阅回调函数
    * 
    * \param msg 话题"vrpn_client_ros/Climbot/pose"消息
    */
    void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    /*!
    * \brief 位姿 速度 加速度订阅回调函数
    * 
    * \param msg 话题"vrpn_client_ros/Climbot/twist"消息
    */
    void NokovCallback(
              const geometry_msgs::PoseStampedConstPtr& pose_msg,
              const geometry_msgs::TwistStampedConstPtr& vel_msg,
              const geometry_msgs::TwistStampedConstPtr& acc_msg);
    
    /*!
    * \brief 设置差分算法
    * 
    * \param algorithm_type 算法类型
    */
    void SetAlgorithm(AlgorithmType algorithm_type = SECOND_ORDER_BWD_DIFF);

    /*!
    * \brief 应用FIR滤波器
    */
    void ApplyFilter();
};

/*!
* \brief 差分算法基类
*/
class DifferentialAlgorithm
{
public:   
    /*!
    * \brief 计算速度的纯虚函数
    * 
    * \param pose_queue 位姿历史队列
    * \param frequency 采样频率
    * \return 计算得到的速度向量
    */
    virtual std::vector<float> Calculate(const std::vector<std::vector<float>>& pose_queue, float frequency) = 0;
};

/*!
* \brief 一阶差分算法实现
*/
class FirstOrderDiff : public DifferentialAlgorithm
{
public:
    std::vector<float> Calculate(const std::vector<std::vector<float>>& pose_queue, float frequency) override;
};

/*!
* \brief 二阶向后差分算法实现
*/
class SecondOrderBackwardDiff : public DifferentialAlgorithm
{
public:
    std::vector<float> Calculate(const std::vector<std::vector<float>>& pose_queue, float frequency) override;
};

#endif
