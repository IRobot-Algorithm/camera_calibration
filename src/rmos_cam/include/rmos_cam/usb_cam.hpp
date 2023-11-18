#ifndef RMOS_CAM__USBCAM_HPP_
#define RMOS_CAM__USBCAM_HPP_

#include "rmos_cam/cam_interface.hpp"

namespace rmos_cam
{
class USBCam : public CamInterface{
public:
    // 打开设备
    bool open() override;

    // 关闭设备
    bool close() override;

    // 返回是否打开
    bool is_open() override;

    // 获取Mat图像
    bool grab_image(cv::Mat &image) override;

    // 获取msg数据，stamp未设置
    bool grab_image(sensor_msgs::msg::Image & image_msg_) override;

    // 设置参数
    bool set_parameter(rmos_cam::CamParamType type, int value) override;

    // 得到参数
    bool get_parameter(rmos_cam::CamParamType type, int &value) override;

    // 返回错误信息
    std::string error_message() override
    {
        return error_message_;
    }

    USBCam(uint cam_id = 0);
    ~USBCam();
private:
    std::string error_message_;
    cv::VideoCapture capture_;
    uint cam_id;
public:
    std::unordered_map<rmos_cam::CamParamType, int> params_;

};

} // namespace rm
#endif