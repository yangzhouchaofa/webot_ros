#include <ros/ros.h>
#include <ros/console.h>
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/RobotMode.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include "std_msgs/Bool.h"
#include "include/wait.h"

#define HOME system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: 0.000000,y: -0.500000,z: 0.300000}, orientation: {x: -1.000000, y: 0.000000,z: 0.000000, w: 0.000000}}, common: {vel: 0.100000, acc: 0.100000, time: 0.0, radius: 0.0}}\"")
float velocity=0.1;
float acceleration=0.1;
bool receive = false;
move_base_msgs::MoveBaseActionGoal pub_msg;
std_msgs::Bool near_car_msg;
std_msgs::Bool pick_msg;


void callback(const geometry_msgs::PoseStamped::ConstPtr& msg, ros::Publisher *pub, ros::Publisher *&near_pub, ros::Publisher *&pick_pub)
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
    if(target_y <= -0.70||target_x>=0.20||target_x<=-0.20)
    {    
        tf::Vector3 pointInLebaiBaseLink(target_x, target_y, target_z);
        tf::TransformListener listener;
        try 
        {
            // Wait for the transform and look it up
            listener.waitForTransform("base_link", "lebai_base_link", ros::Time(0), ros::Duration(3.0));
            tf::StampedTransform transform;
            listener.lookupTransform("base_link", "lebai_base_link", ros::Time(0), transform);

            tf::Vector3 pointInBaseLink = transform * pointInLebaiBaseLink;
            ROS_INFO("Point in base_link: (%f, %f, %f)", pointInBaseLink.x(), pointInBaseLink.y(), pointInBaseLink.z());

            pub_msg.goal.target_pose.header.frame_id = "map";
            if(msg->pose.position.z<0)
            {    
                if(receive == false)
                {
                    pub_msg.goal.target_pose.pose.position.x = pointInBaseLink.x()-0.55;
                    pub_msg.goal.target_pose.pose.position.y = pointInBaseLink.y();
                    pub_msg.goal.target_pose.pose.position.z = pointInBaseLink.z();
                    pub_msg.goal.target_pose.pose.orientation.x = 0;
                    pub_msg.goal.target_pose.pose.orientation.y = 0;
                    pub_msg.goal.target_pose.pose.orientation.z = 0;
                    pub_msg.goal.target_pose.pose.orientation.w= 1;
                }
                pub->publish(pub_msg);
                receive = true;
            }
            ROS_INFO("Point in walk to: (%f, %f, %f)",pub_msg.goal.target_pose.pose.position.x,pub_msg.goal.target_pose.pose.position.y,pub_msg.goal.target_pose.pose.position.z);
        } 
        catch (tf::TransformException& ex) 
        {
            ROS_ERROR("Error transforming point: %s", ex.what());
        }

    }
    else
    {
        if(msg->pose.position.z<0)
        {   
            near_car_msg.data = true;
            int i = 5;
            while(i--)
            {
                near_pub->publish(near_car_msg);
            }
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
            ros::shutdown();
        }
    }

}

int main(int argc, char **argv)
{
    //初始化ros节点
    ros::init(argc, argv, "go_to_position"); 
    //启动机械臂
    system("rosservice call /system_service/enable '{}' "); 
    ros::NodeHandle n; 
    ros::Publisher pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    ros::Publisher near_pub = n.advertise<std_msgs::Bool>("/near_car", 10);
    ros::Publisher pick_pub = n.advertise<std_msgs::Bool>("/pick", 10);
    ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("/target_position",10, boost::bind(callback, _1, &pub, &near_pub, &pick_pub));
    ros::spin();
}