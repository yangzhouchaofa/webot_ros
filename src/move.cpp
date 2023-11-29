#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    move_base_msgs::MoveBaseActionGoal msg;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;  

    // 将欧拉角转换为四元数
    tf::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);

    msg.goal.target_pose.header.frame_id = "map";
    msg.goal.target_pose.pose.position.x = 0.0;
    msg.goal.target_pose.pose.position.y = 0.0;
    msg.goal.target_pose.pose.position.z = 0.0;
    msg.goal.target_pose.pose.orientation.x = quaternion.x();
    msg.goal.target_pose.pose.orientation.y = quaternion.y();
    msg.goal.target_pose.pose.orientation.z = quaternion.z();
    msg.goal.target_pose.pose.orientation.w = quaternion.w();

    ros::Duration(2.0).sleep();  // 休眠2秒

    while (ros::ok()) {
        pub.publish(msg);  // 发布消息
        ROS_INFO("pub");
    //     ros::spinOnce();
    }

    return 0;
}
