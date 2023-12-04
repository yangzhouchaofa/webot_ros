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

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg,  const std::string& topic)
{
    if(topic == "near_car") near_flag = true;
    if(topic == "/target_position")
    {
        if(near_flag)
        {
            double target_x = msg->pose.position.x +0.02;
            double target_y = msg->pose.position.y - 0.025;
            double target_z = -0.075;
            geometry_msgs::Pose arm_target_pose;
            arm_target_pose.position.x= target_x; //给定移动目标相对机械臂底座的空间位置
            arm_target_pose.position.y= target_y;
            arm_target_pose.position.z= target_z;
            arm_target_pose.orientation.x=-1; //给定移动目标相对机械臂底座的空间姿态
            arm_target_pose.orientation.y=0;
            arm_target_pose.orientation.z=0;
            arm_target_pose.orientation.w=0;
            //gohome 
            system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: 0,y: -0.5,z: 0.3}, orientation: {x: -1, y: 0,z: 0, w: 0}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"");
            wait_for_sleep(15);
            std::cout<<"target_pose:"<<arm_target_pose;
            char str[256];
            sprintf(str,"rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: %f,y: %f,z: %f}, orientation: {x: %f, y: %f,z: %f, w: %f}}, common: {vel: 0.1, acc: 0.1, time: 0.0, radius: 0.0}}\"",arm_target_pose.position.x,arm_target_pose.position.y,arm_target_pose.position.z,arm_target_pose.orientation.x,arm_target_pose.orientation.y,arm_target_pose.orientation.z,arm_target_pose.orientation.w);
            system(str);
            wait_for_sleep(20);
            system("rosservice call /io_service/set_gripper_position '{val: 10}'"); 
            wait_for_sleep(5);
            HOME;
            i = 5;
            pick_msg.data = true;
            while(i--)
            {
                pick_pub->publish(pick_msg);
            }
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "near_car"); //初始化ros节点
    ros::NodeHandle n; //Create a node handle //创建节点句柄
    ros::Subscriber near_sub = n.subscribe<std_msgs::Bool>("/near_car", 10, boost::bind(callback, _1,"/near_car"));
    ros::Subscriber target_sub = n.subscribe<geometry_msgs::PoseStamped>("/target_position",10, boost::bind(callback, _1, "/target_position"));

    ros::spin(); // 进入ROS事件循环

    return 0;
}