//
// Created by niuoruo on 8/20/23.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include "cv_bridge/cv_bridge.h"

using namespace std;

class ImagePublisher {

public:

    void publish(cv::Mat& image);

    void rosIn(ros::NodeHandle& n, image_transport::ImageTransport& it);

private:

    // ros::Publisher image_pub_;
    image_transport::Publisher image_pub_;
};