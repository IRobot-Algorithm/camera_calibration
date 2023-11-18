#ifndef RMOS_CAM__CAM_NODE_HPP_
#define RMOS_CAM__CAM_NODE_HPP_

#include <string>
#include <memory>
#include <thread>

#include <opencv2/core.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>

#include "rmos_cam/cam_interface.hpp"
#include "rmos_cam/daheng_cam.hpp"
#include "rmos_cam/usb_cam.hpp"


namespace rmos_cam
{
class CamNode : public rclcpp::Node
{
public:
  CamNode(const std::string & node_name, const rclcpp::NodeOptions & options) : Node(node_name, options) {
    RCLCPP_INFO(this->get_logger(), "Starting node [%s]", node_name.c_str());
  };

protected:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_; // 信息发布
  sensor_msgs::msg::Image image_msg_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
  
  uint32_t frame_id_ = 0;                                       // 帧计数器
};

class DahengCamNode : public virtual CamNode
{
public:
  DahengCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DahengCamNode();



protected:
  std::shared_ptr<DahengCam> cam_dev_;           
  std::thread capture_thread_;                    // 采图线程

};

class USBCamNode : public virtual CamNode
{
public:
  USBCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~USBCamNode();

protected:
  USBCam cam_dev_;
  std::thread capture_thread_;                    // 采图线程
};

} // namespace rmos_cam

#endif // RMOS_CAM__CAM_NODE_HPP_
