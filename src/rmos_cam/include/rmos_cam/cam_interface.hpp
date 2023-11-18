#ifndef RMOS_CAM__CAM_INTERFACE_HPP_
#define RMOS_CAM__CAM_INTERFACE_HPP_

#include <string>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rmos_cam
{
enum class CamParamType
{
  Width,
  Height,
  AutoExposure,
  Exposure,
  Brightness,
  AutoWhiteBalance,
  WhiteBalance,
  Gain,
  RGain,
  GGain,
  BGain,
  Gamma,
  Contrast,
  Saturation,
  Hue,
  Fps
};

// common interface for camera device (usb cam, virtual cam, etc.)
class CamInterface
{
public:
  virtual bool open() = 0;
  virtual bool close() = 0;
  virtual bool is_open() = 0;
  
  virtual bool grab_image(cv::Mat & imgae) = 0;
  virtual bool grab_image(sensor_msgs::msg::Image & image_msg_) = 0;

  // set and get parameter
  virtual bool set_parameter(CamParamType type, int value) = 0;
  virtual bool get_parameter(CamParamType type, int & value) = 0;
  // get error message when above api return false.
  virtual std::string error_message() = 0;
};
} // namespace rm

#endif