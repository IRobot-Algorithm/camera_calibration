//
// Created by niuoruo on 8/20/23.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "mercure_driver.h"
#include "PubImage.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "daheng_camera");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    cv::Mat frame;

    // camera
    int height, width;
    if (!ros::param::get("image_height", height))
    {
        ROS_ERROR("Can not get the value of image_height");
        exit(1);
    }
    if (!ros::param::get("image_width", width))
    {
        ROS_ERROR("Can not get the value of image_width");
        exit(1);
    }

    camera::MercureDriver driver;
    frame.create(height, width, CV_8UC3);

    ImagePublisher ImagePublisherObj;
    ImagePublisherObj.rosIn(n, it);
    
    driver >> frame;
    while (!frame.empty())
    {
        driver >> frame;
        ImagePublisherObj.publish(frame);
        ros::spinOnce();
    }
}