#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vel_node");

    // nh 负责管理话题创建，消息发送
    ros::NodeHandle nh;
    // 规定发布消息对象的主题名和消息对象的容量，这里最多可以接收10个消息
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.1;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        // 发送消息
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    return 0;
}