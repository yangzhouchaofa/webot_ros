// #include "lebai_demo.h"
#include "std_msgs/Bool.h"
#include <move_base_msgs/MoveBaseActionResult.h>
#include "include/wait.h"

bool move_flag = false;


void callback(const std_msgs::Bool::ConstPtr& msg)
{
    if(!move_flag)
    {
        move_flag = true;
        system("rosrun webots_ros move");
    }
}

void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result_msg) {
    // 获取规划结果的状态
    if(move_flag)
    {
        int status = result_msg->status.status;

        // 根据状态判断规划是否成功
        if (status == 3)
        {
            ROS_INFO("Goal reached successfully!");
            system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: 0.58,y: -0.08,z: 0.35}, orientation: {x: 0.5, y: 0.5,z: 0.5, w: 0.5}}, common: {vel: 0.1, acc: 0.1, time: 0.0, radius: 0.0}}\"");
            wait_for_sleep(15);
        } 
        else 
        {
            ROS_INFO("wait to reach the goal. Status: %d", status);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "put"); //初始化ros节点
    ros::NodeHandle n; //Create a node handle //创建节点句柄
    ros::Subscriber pick_up = n.subscribe<std_msgs::Bool>("/pick", 10, callback);
    ros::Subscriber result_sub = n.subscribe("/move_base/result", 10, resultCallback);

    ros::spin(); // 进入ROS事件循环

    return 0;
}
// [ INFO] [1701675307.766054857]: Transform Translation: (0.579155, -0.083193, 0.348318)
// [ INFO] [1701675307.778323304]: Transform Rotation: (0.498582, 0.550727, 0.451455, 0.494272)


