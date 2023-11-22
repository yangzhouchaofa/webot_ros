#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"



void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 在这里可以进行OpenCV图像处理，例如查找色块
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // 定义颜色的范围，这里以红色为例
        cv::Scalar lower_red(0, 100, 100);
        cv::Scalar upper_red(10, 255, 255);

        // 通过颜色范围创建一个二值图像
        cv::Mat mask;
        cv::inRange(hsv_image, lower_red, upper_red, mask);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 遍历轮廓
        for (const auto& contour : contours)
        {
            // 计算轮廓的最小外接矩形
            cv::RotatedRect boundingBox = cv::minAreaRect(contour);

            // 获取矩形的中心坐标
            cv::Point2f center = boundingBox.center;

            // 输出中心坐标
            ROS_INFO("Block Center: (%f, %f)", center.x, center.y);

            // 在图像上绘制矩形
            cv::drawContours(cv_ptr->image, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(0, 255, 0), 2);
        }

        // 在这里可以进行其他处理，例如显示图像
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


