#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "tf/transform_listener.h"


// 全局变量用于存储TransformListener
tf::TransformListener* listener;
// 添加一个标志，表示是否已经收到了RGB图像
bool received_rgb = false;
sensor_msgs::Image::ConstPtr rgb_msg;
sensor_msgs::Image::ConstPtr depth_msg;

// 相机内参
const double fx = 606.31005859375;  // 以实际值替换
const double fy = 606.111572265625;  // 以实际值替换
const double cx = 324.8408508300781;  // 以实际值替换
const double cy = 248.92527770996094;  // 以实际值替换                    

void imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic)
{
    if (topic == "/camera/color/image_raw")
    {
        // 如果是RGB图像，将其保存，并设置标志为true
        rgb_msg = msg;
        received_rgb = true;
    }
    else if (topic == "/camera/aligned_depth_to_color/image_raw")
    {
        // 如果是深度图像，将其保存
        depth_msg = msg;

        // 如果已经收到了RGB图像，调用处理函数
        if (received_rgb)
        {
            try
            {
                // 将ROS RGB图像 深度图像 消息转换为OpenCV图像
                cv_bridge::CvImagePtr cv_ptr_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
                cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

                // 在这里可以进行OpenCV图像处理，例如查找色块
                cv::Mat hsv_image;
                cv::cvtColor(cv_ptr_rgb->image, hsv_image, cv::COLOR_BGR2HSV);

                // 定义颜色的范围，这里以红色为例
                cv::Scalar lower_red(0, 100, 100);
                cv::Scalar upper_red(10, 255, 255);

                // 通过颜色范围创建一个二值图像
                cv::Mat mask;
                cv::inRange(hsv_image, lower_red, upper_red, mask);

                // 查找轮廓
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                ROS_INFO("-----------------------------------------------------------------");

                float x=0,  y=0, i=0;

                for (const auto& contour : contours)
                {
                     // 计算轮廓的最小外接矩形
                    cv::RotatedRect boundingBox = cv::minAreaRect(contour);

                     if(cv::contourArea(contour)>=10)
                    {

                        // 获取矩形的中心坐标
                        cv::Point2f center = boundingBox.center;
                        cv::drawContours(cv_ptr_rgb->image, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(0, 255, 0), 2);
                        x+=center.x;
                        y+=center.y;
                        ROS_INFO("Block Center: (%f, %f)", center.x,center.y);
                        i++;
                    }        
                }                
                        x = x/i;
                        y = y/i;

                    // 获取深度值
                    float depth_value = cv_ptr_depth->image.at<float>(y, x);
                    ROS_INFO("矩形的中心坐标: (%f, %f),深度值: (%f)",x, y, depth_value);

                    // 计算相机坐标系下的坐标
                    double camera_x = (x - cx) * depth_value / fx;
                    double camera_y = (y - cy) * depth_value / fy;
                    double camera_z = depth_value;
                    ROS_INFO("相机坐标系下的坐标: (%f, %f, %f)", camera_x, camera_y, camera_z);

                    // 将点从相机坐标系转换到机器人坐标系
                    tf::StampedTransform transform;
                    try
                    {
                        listener->waitForTransform("lebai_base_link", "camera_aligned_depth_to_color_frame", ros::Time(0), ros::Duration(3.0));
                        listener->lookupTransform("lebai_base_link", "camera_aligned_depth_to_color_frame", ros::Time(0), transform);
                    }
                    catch (tf::TransformException& ex)
                    {
                        ROS_ERROR("Error transforming point: %s", ex.what());
                        return;
                    }

                    // 创建点的Homogeneous Coordinates（齐次坐标）
                    tf::Vector3 point_camera(camera_x/1000.0, camera_y/1000.0, camera_z/1000.0);

                    // 将点从相机坐标系转换到机器人坐标系
                    tf::Vector3 point_robot = transform * point_camera;

                    // 输出中心坐标和深度值
                    ROS_INFO("机器人坐标系: (%f, %f, %f)", point_robot.x(), point_robot.y(), point_robot.z());

                // 处理完毕后，重置标志
                received_rgb = false;

                // 在这里可以进行其他处理，例如显示图像
                cv::imshow("Received Image", cv_ptr_rgb->image);
                cv::waitKey(1);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image");
    ros::NodeHandle nh;
    
    // 创建TransformListener
    listener = new tf::TransformListener();
    system("rosservice call /io_service/set_gripper_position '{val: 100}'");

    // 订阅RGB图像消息
    ros::Subscriber rgb_sub = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, boost::bind(imageCallback, _1, "/camera/color/image_raw"));

    // 订阅深度图像消息
    ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 1, boost::bind(imageCallback, _1, "/camera/aligned_depth_to_color/image_raw"));
    
    ros::spin();

    return 0;
}



//   frame_id: "camera_color_optical_frame"
// height: 480
// width: 640
// K: [606.31005859375, 0.0, 324.8408508300781, 0.0, 606.111572265625, 248.92527770996094, 0.0, 0.0, 1.0]
