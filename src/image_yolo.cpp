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


// 全局变量用于存储TransformListener
tf::TransformListener* listener;
// 添加一个标志，表示是否已经收到了RGB图像
bool received_rgb = false;
bool near_car = false;
sensor_msgs::Image::ConstPtr depth_msg;
std_msgs::Bool flag_msg;
float x = 700,y = 700;
bool target_find = false;

// 相机内参
const double fx = 606.31005859375;  
const double fy = 606.111572265625;  
const double cx = 324.8408508300781; 
const double cy = 248.92527770996094;

void YoloCallback(const geometry_msgs::Point::ConstPtr& msg){
        x = msg -> x;
        y = msg -> y;
    if (x == 700 && y == 700){
        target_find = false;}
    else{
        target_find = true;
        }}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic, ros::Publisher *target_pub){
    if (target_find){
        depth_msg = msg;
        cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        float depth_value = cv_ptr_depth->image.at<float>(y, x);
        // ROS_INFO("矩形的中心坐标: (%d, %d),深度值: (%f)",x, y, depth_value);

        // 计算相机坐标系下的坐标
        double camera_x = ((x - cx) * depth_value / fx)/1000.0;
        double camera_y = ((y - cy) * depth_value / fy)/1000.0;
        double camera_z = depth_value/1000.0;
        // ROS_INFO("相机坐标系下的坐标: (%f, %f, %f)", camera_x, camera_y, camera_z);

        // 将点从相机坐标系转换到机器人坐标系
        tf::StampedTransform transform;
        try{
            listener->waitForTransform("lebai_base_link", "camera_aligned_depth_to_color_frame", ros::Time(0), ros::Duration(3.0));
            listener->lookupTransform("lebai_base_link", "camera_aligned_depth_to_color_frame", ros::Time(0), transform);}
        catch (tf::TransformException& ex){
            ROS_ERROR("Error transforming point: %s", ex.what());
            return;}

        tf::Vector3 point_camera(camera_x, camera_y, camera_z); // 创建点的Homogeneous Coordinates（齐次坐标）
        tf::Vector3 point_robot = transform * point_camera;     // 将点从相机坐标系转换到机器人坐标系

        // 输出中心坐标和深度值
//         ROS_INFO("机器人坐标系: (%f, %f, %f)", point_robot.x(), point_robot.y(), point_robot.z());

        // 设置发布的消息
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "lebai_base_link";  // 坐标系，可以根据实际情况修改
        target_pose.pose.position.x = point_robot.x();    // 目标位置的X坐标
        target_pose.pose.position.y = point_robot.y();    // 目标位置的Y坐标
        target_pose.pose.position.z = 0;    // 目标位置的Z坐标
        target_pose.pose.orientation.x = 0.0; // 方向四元数，可以根据实际情况修改
        target_pose.pose.orientation.y = 0.0;
        target_pose.pose.orientation.z = 0.0;
        target_pose.pose.orientation.w = 1.0;

        target_pub->publish(target_pose);}}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_yolo");
    ros::NodeHandle nh;

   listener = new tf::TransformListener();
    system("rosservice call /system_service/enable '{}' "); //启动机械臂
    system("rosservice call /io_service/set_gripper_position '{val: 100}'");
   tf::StampedTransform transform;
   try{
       listener->waitForTransform("lebai_base_link", "camera_aligned_depth_to_color_frame", ros::Time(0), ros::Duration(3.0));
       listener->lookupTransform("lebai_base_link", "camera_aligned_depth_to_color_frame", ros::Time(0), transform);
       ROS_INFO("Transform values: Translation(x=%f, y=%f, z=%f), Rotation(x=%f, y=%f, z=%f, w=%f)",
                           transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(),
                           transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());}
   catch (tf::TransformException& ex){
       ROS_ERROR("Error transforming point: %s", ex.what());}
   if(transform.getOrigin().x()<0&&transform.getOrigin().x()>-0.2&&transform.getOrigin().y()>-0.1&&transform.getOrigin().y()<0.1&&transform.getOrigin().z()>0.5&&transform.getOrigin().z()<0.7){
       ROS_INFO("move already");}
   else{
//       system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: 0,y: -0.4,z: 0.4}, orientation: {x: 0.5, y: 0,z: 0, w: 0.5}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"");
//       wait_for_sleep(7);
       system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: -0.1,y: 0.0,z: 0.58}, orientation: {x: 0.88, y: 0.0,z: -0.0, w: 0.46}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"");
       wait_for_sleep(15);}

    ros::Publisher target_pub = nh.advertise<geometry_msgs::PoseStamped>("/target_position", 10);
    ros::Subscriber yolo_sub = nh.subscribe<geometry_msgs::Point>("/yolo_bottle", 1, YoloCallback);
    ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 1, boost::bind(imageCallback, _1, "/camera/aligned_depth_to_color/image_raw", &target_pub));
    ros::spin();
    return 0;
}



//   frame_id: "camera_color_optical_frame"
// height: 480
// width: 640
// K: [606.31005859375, 0.0, 324.8408508300781, 0.0, 606.111572265625, 248.92527770996094, 0.0, 0.0, 1.0]
//  system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: -0.121547,y: 0.008318,z: 0.421518}, orientation: {x: 0.924148, y: 0.012173,z: -0.025137, w: 0.381012}}, common: {vel: 0.1, acc: 0.1, time: 0.0, radius: 0.0}}\"");
