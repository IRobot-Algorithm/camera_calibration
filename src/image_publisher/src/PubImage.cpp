#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "cv_bridge/cv_bridge.h"
#include "PubImage.h"

using namespace std;

void ImagePublisher::rosIn(ros::NodeHandle& n, image_transport::ImageTransport& it)
{
    image_pub_ = it.advertise("image_raw", 1);
}

void ImagePublisher::publish(cv::Mat& image)
{
    // 创建图像消息
    sensor_msgs::ImagePtr msg = 
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        msg->header.frame_id = "Image";
        msg->header.seq++;
        msg->header.stamp = ros::Time::now();

    // 发布图像消息
    image_pub_.publish(msg);

}

