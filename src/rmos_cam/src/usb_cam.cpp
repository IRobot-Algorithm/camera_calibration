#include "rmos_cam/usb_cam.hpp"

namespace rmos_cam
{
    USBCam::USBCam(uint cam_id) : cam_id(cam_id)
    {
    }

    USBCam::~USBCam()
    {
        this->close();
    }

    bool USBCam::grab_image(cv::Mat &image)
    {
        if (is_open())
        {
            capture_ >> image;
            return true;
        }
        return false;
    }

    bool USBCam::grab_image(sensor_msgs::msg::Image &image_msg_)
    {
        if (is_open())
        {
            cv::Mat image;
            capture_ >> image;
            image_msg_.encoding = "bgr8";
            image_msg_.height = image.rows;
            image_msg_.width = image.cols;
            image_msg_.step = image.cols * 3;
            image_msg_.header.frame_id = "USBCam";

            image_msg_.data.reserve(image.rows * image.cols * 3);
            image_msg_.data.resize(image.rows * image.cols * 3);
            memcpy(image_msg_.data.data(), image.data, image.rows * image.cols * 3);
            return true;
        }
        return false;
    }

    bool USBCam::is_open()
    {
        return capture_.isOpened();
    }

    bool USBCam::open()
    {
        capture_ = cv::VideoCapture(cam_id);
        return capture_.isOpened();
    }

    bool USBCam::close()
    {
        capture_.release();
        return true;
    }

    bool USBCam::set_parameter(CamParamType type, int value)
    {
        params_[type] = value;
        return true;
    }

    // 得到参数
    bool USBCam::get_parameter(CamParamType type, int &value)
    {
        if (params_.find(type) != params_.end())
        {
            value = params_[type];
            return true;
        }
        else
        {
            this->error_message_ = "USBCam get_Parameter failed";
            return false;
        }
    }

} // namespace rmos_cam