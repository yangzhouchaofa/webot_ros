#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"



void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO("This is an INFO message");
    try
    {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 在这里可以进行OpenCV图像处理，例如显示图像
        cv::imshow("Received Image", cv_ptr->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image");
    ros::NodeHandle nh;

    // 创建一个订阅器来接收图像数据
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);

    // 初始化OpenCV窗口（如果需要显示图像）
    cv::namedWindow("Received Image");

    // 进入ROS循环
    ros::spin();

    // 关闭OpenCV窗口
    cv::destroyWindow("Received Image");

    return 0;
}


