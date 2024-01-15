#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include "std_msgs/Bool.h"
#include "include/wait.h"

bool near_flag = false;
bool find_flag = false;
bool move_flag = false;
int position = 0;

void nearcallback(const std_msgs::Bool::ConstPtr& msg)
{
        near_flag = msg ->data;
        ROS_INFO("sub near_car_msg");
}

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg, ros::Publisher *&pick_pub)
{
    if(near_flag&&(msg->pose.position.y <= 0)&&(msg->pose.position.y >= -1))
    {
        double target_x = msg->pose.position.x;
        double target_y = msg->pose.position.y +0.11;//+13
        double target_z = 0.34;//0.26
        geometry_msgs::Pose arm_target_pose;
        arm_target_pose.position.x= target_x; //给定移动目标相对机械臂底座的空间位置
        arm_target_pose.position.y= target_y;
        arm_target_pose.position.z= target_z;
        arm_target_pose.orientation.x=0.5; //给定移动目标相对机械臂底座的空间姿态
        arm_target_pose.orientation.y=0;
        arm_target_pose.orientation.z=0;
        arm_target_pose.orientation.w=0.5;
        ROS_INFO("near&&find");
        ROS_INFO("pick");
        std::cout<<"target_pose:"<<arm_target_pose;
        char str[256];

        std::cout<<"target_pose:"<<arm_target_pose;
//        sprintf(str,"rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: %f,y: %f,z: %f}, orientation: {x: 0.5, y: 0,z: 0, w: 0.5}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"",arm_target_pose.position.x,arm_target_pose.position.y,arm_target_pose.position.z);
        sprintf(str,"rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: %f,y: %f,z: 0.45}, orientation: {x: 0.88, y: 0.0,z: -0.0, w: 0.46}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"",arm_target_pose.position.x,arm_target_pose.position.y);
        system(str);

        sprintf(str,"rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: %f,y: %f,z: %f}, orientation: {x: 0.88, y: 0.0,z: -0.0, w: 0.46}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"",arm_target_pose.position.x,arm_target_pose.position.y,arm_target_pose.position.z);
        system(str);
        std::cout<<str<<std::endl;
        wait_for_sleep(15);

        system("rosservice call /io_service/set_gripper_position '{val: 10}'");
        wait_for_sleep(3);

        system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: -0.1,y: 0.0,z: 0.58}, orientation: {x: 0.88, y: 0.0,z: -0.0, w: 0.46}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"");
        wait_for_sleep(15);

        std_msgs::Bool pick_msg;
        pick_msg.data = true;
        while(ros::ok())
        {
            pick_pub->publish(pick_msg);
        }
    }
}

void targetcallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(near_flag)
    {
        find_flag = msg ->data;
        switch (position)
        {
            case 0:
                    HOME;
                    ROS_INFO("go to home");
                    position++;
                    wait_for_sleep(15);
                break;
            // case 1:
            //         POSITION1;
            //         ROS_INFO("go to position1");
            //         position++;
            //         wait_for_sleep(5);
            //     break;
            // case 2:
            //         POSITION2;
            //         ROS_INFO("go to position2");
            //         position++;
            //         wait_for_sleep(5);
            //     break;                
            // case 3:
            //         POSITION3;
            //         ROS_INFO("go to position3");
            //         position++;
            //         wait_for_sleep(5);
            //     break;            
            // case 4:
            //         POSITION4;
            //         ROS_INFO("go to position4");
            //         position++;
            //         wait_for_sleep(5);
                break;          
            default:
                break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "near_car"); //初始化ros节点
    ros::NodeHandle n; //Create a node handle //创建节点句柄
    ros::Publisher pick_pub = n.advertise<std_msgs::Bool>("/pick", 10); //
    ros::Subscriber near_sub = n.subscribe<std_msgs::Bool>("/near_car", 10, nearcallback);
    ros::Subscriber target_sub = n.subscribe<geometry_msgs::PoseStamped>("/target_position",10, boost::bind(callback,  _1, &pick_pub));
    ros::Subscriber find_sub = n.subscribe<std_msgs::Bool>("/target_find", 10, targetcallback);
    ros::spin(); // 进入ROS事件循环

    return 0;
}