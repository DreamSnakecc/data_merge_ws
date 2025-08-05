#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "vrpn_node");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("vrpn_topic", 10);

    ros::Rate loop_rate(5); 

    float i = 0;
    while(ros::ok())
    {
        if(i > 100)
        {
            i = 0;
        }
        i++;
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.pose.position.x = i;
        msg.pose.position.y = i + 10;
        msg.pose.position.z = i + 20;
        msg.pose.orientation.w = i * 10;
        msg.pose.orientation.x = i * 20;
        msg.pose.orientation.y = i * 30;
        msg.pose.orientation.z = i * 40;
        pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}