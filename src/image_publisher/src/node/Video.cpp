//
// Created by niuoruo on 8/20/23.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "PubImage.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    cv::Mat frame;


    // video
    std::string video_path;
    if (!ros::param::get("video_path", video_path))
    {
        ROS_ERROR("Can not get the value of video_path");
        exit(1);
    }
    cv::VideoCapture cap(video_path);

    ImagePublisher ImagePublisherObj;
    ImagePublisherObj.rosIn(n, it);
    
    cap >> frame;
    while (!frame.empty())
    {
        ImagePublisherObj.publish(frame);
        cap >> frame;
        ros::spinOnce();
    }
}