#include "rmos_cam/cam_node.hpp"

namespace rmos_cam
{
    USBCamNode::USBCamNode(const rclcpp::NodeOptions &options) : CamNode("usb_camera", options)
    {
        cam_dev_ = USBCam();

        cam_dev_.open();

        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);   

        capture_thread_ = std::thread{[this]() -> void
                                      {
                                          RCLCPP_INFO(this->get_logger(), "Publishing image!");

                                          while (rclcpp::ok())
                                          {
                                              cv::Mat frame_;
                                              cv_bridge::CvImage bridge_;
                                              if (cam_dev_.grab_image(frame_))
                                              {
                                                  bridge_.image = frame_;
                                                  bridge_.header.frame_id = "USBCam" + std::to_string(this->frame_id_++);
                                                  bridge_.encoding = "bgr8";

                                                  img_pub_->publish(*bridge_.toImageMsg());
                                              }
                                          }
                                      }};
    }

    USBCamNode::~USBCamNode()
    {
        if (capture_thread_.joinable())
        {
            capture_thread_.join();
        }
        cam_dev_.close();
        RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
    }

} // namespace rmos_cam
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::USBCamNode)