#include <ros/ros.h>
#include <ros/console.h>
// #include <industrial_msgs/RobotStatus.h>
// #include <industrial_msgs/RobotMode.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include "std_msgs/Bool.h"
#include "include/wait.h"

bool near_flag = false;
bool find_flag = false;
int position = 0;
geometry_msgs::Twist vel_msg;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel"); //初始化ros节点
    ros::NodeHandle n; //Create a node handle //创建节点句柄
    ros::Publisher vel_cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ROS_INFO("move");    
    vel_msg.linear.x = -0.13 ;
    vel_msg.angular.z = 0.0;
    // vel_msg.angular.z = - msg->pose.position.x/10;
    // while (ros::ok()){vel_cmd_pub.publish(vel_msg);}
    // sleep(1);
    int i = 10000;
    while (i--)
    {
        vel_cmd_pub.publish(vel_msg);
        sleep(0.0001);
    }
    
    vel_msg.linear.x =  0.0 ;
    vel_msg.angular.z = 0.0;
    vel_cmd_pub.publish(vel_msg);


    return 0;
}