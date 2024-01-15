#include <ros/ros.h>
#include <ros/console.h>
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/RobotMode.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>

int main(int argc, char **argv)
{
    char str[256];
    float velocity=0.1;
    float acceleration=0.1;
    ros::init(argc, argv, "home"); //初始化ros节点
    system("rosservice call /system_service/enable '{}' "); //启动机械臂
    system("rosservice call /io_service/set_gripper_position '{val: 100}'");

    ros::NodeHandle n; 
    geometry_msgs::Pose arm_target_pose;
    while(ros::ok())
    {
            arm_target_pose.position.x = 0; //给定移动目标相对机械臂底座的空间位置
            arm_target_pose.position.y = -0.5;
            arm_target_pose.position.z = 0.4;
            arm_target_pose.orientation.x = 0.5; //给定移动目标相对机械臂底座的空间姿态
            arm_target_pose.orientation.y = 0;
            arm_target_pose.orientation.z = 0;
            arm_target_pose.orientation.w= 0.5;
            std::cout<<"target_pose:"<<arm_target_pose<<std::endl;
            sprintf(str,"rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: %f,y: %f,z: %f}, orientation: {x: %f, y: %f,z: %f, w: %f}}, common: {vel: %f, acc: %f, time: 0.0, radius: 0.0}}\"",arm_target_pose.position.x,arm_target_pose.position.y,arm_target_pose.position.z,arm_target_pose.orientation.x,arm_target_pose.orientation.y,arm_target_pose.orientation.z,arm_target_pose.orientation.w,velocity,acceleration);
            std::cout<<str;
            system(str);
            ros::shutdown(); 

            return 0;
        
        ros::spinOnce();
    }
}