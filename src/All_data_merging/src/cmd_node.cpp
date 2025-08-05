#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <All_data_merging/cmd.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "cmd_node");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<All_data_merging::cmd>("/cmd_data", 3);

    ros::Rate loop_rate(50); 

    float i = 0;
    while(ros::ok())
    {
        if(i > 100)
        {
            i = 0;
        }
        i++;
        All_data_merging::cmd msg;
        msg.command = "Command " + std::to_string(i);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.sec_f = (double)(msg.header.stamp.nsec)/1e9 + msg.header.stamp.sec;
        pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}