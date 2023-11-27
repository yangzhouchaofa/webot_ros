#include <ros/ros.h>
#include <ros/console.h>
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/RobotMode.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>

float velocity=0.1;
float acceleration=0.1;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("111");
    double target_x = msg->pose.position.x;
    double target_y = msg->pose.position.y;
    double target_z = -0.075;
    geometry_msgs::Pose arm_target_pose;
    arm_target_pose.position.x= target_x; //给定移动目标相对机械臂底座的空间位置
    arm_target_pose.position.y= target_y;
    arm_target_pose.position.z= target_z;
    arm_target_pose.orientation.x=-1; //给定移动目标相对机械臂底座的空间姿态
    arm_target_pose.orientation.y=0;
    arm_target_pose.orientation.z=0;
    arm_target_pose.orientation.w=0;
    std::cout<<"target_pose:"<<arm_target_pose;
    char str[256];
    sprintf(str,"rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: %f,y: %f,z: %f}, orientation: {x: %f, y: %f,z: %f, w: %f}}, common: {vel: %f, acc: %f, time: 0.0, radius: 0.0}}\"",arm_target_pose.position.x,arm_target_pose.position.y,arm_target_pose.position.z,arm_target_pose.orientation.x,arm_target_pose.orientation.y,arm_target_pose.orientation.z,arm_target_pose.orientation.w,velocity,acceleration);
    system(str);

    sleep(20);
    system("rosservice call /io_service/set_gripper_position '{val: 10}'"); 
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_to_position"); //初始化ros节点
    system("rosservice call /system_service/enable '{}' "); //启动机械臂
    ros::NodeHandle n; 
    ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("/target_position",10,callback);
    ros::spin();
}