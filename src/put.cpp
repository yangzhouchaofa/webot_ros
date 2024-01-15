#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "tf/transform_listener.h"
#include "include/lebai_demo.h"
#include "std_msgs/Bool.h"
#include "include/wait.h"
#include "geometry_msgs/Point.h"
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>


// 添加一个标志，表示是否已经收到了RGB图像
bool receive = false;
bool received_rgb = false;
bool near_car = false;
sensor_msgs::Image::ConstPtr depth_msg;
std_msgs::Bool flag_msg;
float x = 700,y = 700;
bool target_find = false;
float x_map = 10 , y_map = 10 , z_map = 10;
move_base_msgs::MoveBaseActionGoal pub_msg;

// 相机内参
const double fx = 606.31005859375;
const double fy = 606.111572265625;
const double cx = 324.8408508300781;
const double cy = 248.92527770996094;

void Yolowastebin(const geometry_msgs::Point::ConstPtr& msg){
        x = msg -> x;
        y = msg -> y;
    if (x == 700 && y == 700){
        target_find = false;}
    else{
        target_find = true;
        }}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic, ros::Publisher *target_pub, ros::Publisher *pub){
    if (target_find){
        depth_msg = msg;
        cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        float depth_value = cv_ptr_depth->image.at<float>(y, x);
        // ROS_INFO("矩形的中心坐标: (%d, %d),深度值: (%f)",x, y, depth_value);

        // 计算相机坐标系下的坐标
        double camera_x = ((x - cx) * depth_value / fx)/1000.0;
        double camera_y = ((y - cy) * depth_value / fy)/1000.0;
        double camera_z = depth_value/1000.0;
//         ROS_INFO("相机坐标系下的坐标: (%f, %f, %f)", camera_x, camera_y, camera_z);

        // 将点从相机坐标系转换到机器人坐标系
        tf::StampedTransform transform;
        tf::TransformListener listener;
        try{
            listener.waitForTransform("map", "camera_aligned_depth_to_color_frame", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("map", "camera_aligned_depth_to_color_frame", ros::Time(0), transform);}
        catch (tf::TransformException& ex){
            ROS_ERROR("Error transforming point: %s", ex.what());
            return;}

        tf::Vector3 point_camera(camera_x, camera_y, camera_z); // 创建点的Homogeneous Coordinates（齐次坐标）
        tf::Vector3 point_robot = transform * point_camera;     // 将点从相机坐标系转换到机器人坐标系

        // 输出中心坐标和深度值
        ROS_INFO("机器人坐标系: (%f, %f, %f)", point_robot.x(), point_robot.y(), point_robot.z());
        if((point_robot.z()>0)&&(point_robot.z()<0.5)){
            x_map = point_robot.x();
            y_map = point_robot.y();
            z_map = point_robot.z();
//            move_base_msgs::MoveBaseActionGoal pub_msg;
//            pub_msg.goal.target_pose.header.frame_id = "map";
//            pub_msg.goal.target_pose.pose.position.x = x_map - 0.6;
//            pub_msg.goal.target_pose.pose.position.y = y_map;
//            pub_msg.goal.target_pose.pose.position.z = 0;
//            pub_msg.goal.target_pose.pose.orientation.x = 0;
//            pub_msg.goal.target_pose.pose.orientation.y = 0;
//            pub_msg.goal.target_pose.pose.orientation.z = 0;
//            pub_msg.goal.target_pose.pose.orientation.w= 1;
//            pub->publish(pub_msg);
            }
        }}

void pickcallback(const std_msgs::Bool::ConstPtr& msg, ros::Publisher *pub)
{
     pub_msg.goal.target_pose.header.frame_id = "map";
     if(receive == false){
         pub_msg.goal.target_pose.pose.position.x = x_map - 0.6;
         pub_msg.goal.target_pose.pose.position.y = y_map;
         pub_msg.goal.target_pose.pose.position.z = 0;
         pub_msg.goal.target_pose.pose.orientation.x = 0;
         pub_msg.goal.target_pose.pose.orientation.y = 0;
         pub_msg.goal.target_pose.pose.orientation.z = 0;
         pub_msg.goal.target_pose.pose.orientation.w= 1;
     }
     pub->publish(pub_msg);
     receive = true;
}

void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result_msg)
{
    if(receive)
    {
        int status = result_msg->status.status;
        // 根据状态判断规划是否成功
        if (status == 3)
        {
            ros::shutdown();
        }
        else
        {
            ROS_INFO("wait to reach the goal. Status: %d", status);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "put");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    ros::Subscriber near_sub = nh.subscribe<std_msgs::Bool>("/pick", 10, boost::bind(pickcallback, _1, &pub));
    ros::Publisher target_pub = nh.advertise<geometry_msgs::PoseStamped>("/wastebin_position", 10);
    ros::Subscriber yolo_sub = nh.subscribe<geometry_msgs::Point>("/yolo_wastebin", 1, Yolowastebin);
    ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 1, boost::bind(imageCallback, _1, "/camera/aligned_depth_to_color/image_raw", &target_pub, &pub));
    ros::Subscriber result_sub = nh.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 10, resultCallback);
    ros::spin();
    return 0;
}
// [ INFO] [1701675307.766054857]: Transform Translation: (0.579155, -0.083193, 0.348318)
// [ INFO] [1701675307.778323304]: Transform Rotation: (0.498582, 0.550727, 0.451455, 0.494272)


