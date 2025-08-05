#include "NOKOV2ROS/get_pose_vel.h"
#include "NOKOV2ROS/robot_pose_vel.h"

// ===== GET_POSE_VEL 类实现 =====

GET_POSE_VEL::GET_POSE_VEL(ros::NodeHandle &nh, std::string climbot_name, const float &frequency, AlgorithmType algorithm_type)
    : nh_(nh), climbot_name_(climbot_name), frequency_(frequency), diff_algorithm_(nullptr), pose_queue_size_(3)
{
    // 初始化位置、四元数和速度
    pose_.resize(6, 0.0);
    q4_.setIdentity();  
    curr_vel_.resize(6, 0.0);

    // 设置差分算法（这会更新 pose_queue_size_）
    SetAlgorithm(algorithm_type);
    
    // 重新调整位姿队列大小（基于算法需求）
    pose_queue_.clear();
    pose_queue_.resize(pose_queue_size_, pose_);
    curr_vel_queue_.resize(26, curr_vel_);

    //vrpn发布的话题名称格式，climbot_name是XINGYING中定义的刚体名称
    std::string topic_name = "vrpn_client_ros/" + climbot_name + "/pose";
    // pose_sub_ = nh_.subscribe(topic_name, 1, &GET_POSE_VEL::PoseCallback, this);   //实际代码
    pose_sub_ = nh_.subscribe("vrpn_topic", 1, &GET_POSE_VEL::PoseCallback, this);    //测试用

    // 初始化发布器
    pose_vel_pub_ = nh_.advertise<NOKOV2ROS::robot_pose_vel>("/pose_vel_data", 1);
}

void GET_POSE_VEL::SetAlgorithm(AlgorithmType algorithm_type)
{
    switch (algorithm_type) {
        case FIRST_ORDER_DIFF:
            diff_algorithm_ = new FirstOrderDiff();
            pose_queue_size_ = 2; // 一阶差分只需要前后两个数据
            break;
        case SECOND_ORDER_BWD_DIFF:
            diff_algorithm_ = new SecondOrderBackwardDiff();
            pose_queue_size_ = 3; // 二阶向后差分需要至少三个数据
            break;
    }
    ROS_WARN("%s", algorithm_type == FIRST_ORDER_DIFF ? "Switched to First Order Diff\n" : "Switched to Second Order Backward Diff\n");
}

void GET_POSE_VEL::CalculateVel()
{
    // 使用当前算法计算速度
    curr_vel_ = diff_algorithm_ -> Calculate(pose_queue_, frequency_);
    // 应用滤波
    ApplyFilter();
}

void GET_POSE_VEL::ApplyFilter()
{
    curr_vel_queue_.erase(curr_vel_queue_.begin());
    curr_vel_queue_.push_back(curr_vel_);            //更新速度序列

    std::vector<float> filter_vel_temp(6, 0.0); //滤波计算过渡数组
    
    //滤波
    for (size_t i = 0; i < 26; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            filter_vel_temp[j] += fir_a_[i] * curr_vel_queue_[i][j];
        }
    }
    curr_vel_ = filter_vel_temp; //得到滤波后速度
}

void GET_POSE_VEL::PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    // 更新机器人位置
    pose_[0] = msg->pose.position.x;
    pose_[1] = msg->pose.position.y;
    pose_[2] = msg->pose.position.z;

    // 更新四元数
    q4_.x() = msg->pose.orientation.x;
    q4_.y() = msg->pose.orientation.y;
    q4_.z() = msg->pose.orientation.z;
    q4_.w() = msg->pose.orientation.w;

    //计算欧拉角
    Eigen::Vector3f euler_angles = q4_.toRotationMatrix().eulerAngles(0, 1, 2);
    pose_[3] = euler_angles[0]; // Roll
    pose_[4] = euler_angles[1]; // Pitch
    pose_[5] = euler_angles[2]; // Yaw

    // 更新位姿队列
    pose_queue_.erase(pose_queue_.begin());
    pose_queue_.push_back(pose_); // 更新位姿序列

    this -> CalculateVel(); // 计算速度

    // 发布位姿和速度
    NOKOV2ROS::robot_pose_vel pose_vel_msg;

    pose_vel_msg.nokov_pose = *msg;
    for (size_t i = 0; i < 6; ++i) 
    {
        pose_vel_msg.curr_vel[i] = this->curr_vel_[i];
    }
    pose_vel_msg.sec_f = (double)(msg->header.stamp.nsec)/1e9 + msg->header.stamp.sec;
    pose_vel_msg.header = msg->header;

    pose_vel_pub_.publish(pose_vel_msg);
}

// ===== 差分算法实现 =====

std::vector<float> FirstOrderDiff::Calculate(const std::vector<std::vector<float>>& pose_queue, float frequency)
{
    std::vector<float> velocity(6, 0.0);   
    // 一阶差分
    const std::vector<float>& pose_current = pose_queue[1];      // 当前位姿
    const std::vector<float>& pose_previous = pose_queue[0]; // 前一个位姿
    
    for (size_t i = 0; i < 6; i++) {
        velocity[i] = (pose_current[i] - pose_previous[i]) * frequency;
    }
    return velocity;
}

std::vector<float> SecondOrderBackwardDiff::Calculate(const std::vector<std::vector<float>>& pose_queue, float frequency)
{
    std::vector<float> velocity(6, 0.0);
    // 二阶向后差分
    const std::vector<float>& pose_1 = pose_queue[2];        
    const std::vector<float>& pose_2 = pose_queue[1]; 
    const std::vector<float>& pose_3 = pose_queue[0];  
    
    for (size_t i = 0; i < 6; i++) {
        velocity[i] = (3 * pose_1[i] - 4 * pose_2[i] + pose_3[i]) / 2 / frequency;
    }
    return velocity;
}
