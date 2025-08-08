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

    // 创建GET_POSE_VEL对象，传入指定的算法类型
    pose_vel = new GET_POSE_VEL(nh, climbot_name, 40);
    
    ros::spin();
    
    delete pose_vel;
    return 0;
}
