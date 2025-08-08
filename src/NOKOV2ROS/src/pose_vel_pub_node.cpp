#include "ros/ros.h"
#include "NOKOV2ROS/get_pose_vel.h"
#include "NOKOV2ROS/robot_pose_vel.h"

GET_POSE_VEL *pose_vel;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_vel_pub_node");     //初始化ROS节点
    if (argc != 2)
    {
        ROS_INFO("Please set climbot_name");
        return 1;
    }
    std::string climbot_name = argv[1];
    ROS_WARN(" %s", climbot_name.c_str());
    
    ros::NodeHandle nh;
    ros::Rate loop_rate(40);

    //选择速度算法
    GET_POSE_VEL::AlgorithmType algorithm_type = GET_POSE_VEL::SECOND_ORDER_BWD_DIFF; // 默认使用二阶向后差分
    // 创建GET_POSE_VEL对象，传入指定的算法类型
    pose_vel = new GET_POSE_VEL(nh, climbot_name, 40, algorithm_type);
    
    ros::spin();
    
    delete pose_vel;
    return 0;
}
