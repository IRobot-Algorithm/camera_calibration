#include "rmos_cam/cam_node.hpp"


#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>


#include <chrono>


namespace rmos_cam
{
    DahengCamNode::DahengCamNode(const rclcpp::NodeOptions &options) : CamNode("daheng_camera", options)
    {
        // cam dev
        cam_dev_ = std::make_shared<DahengCam>();

        // paramter
        int Width, Height, Exposure, RGain, GGain, BGain, Gamma, Fps;
        int AutoExposure, AutoWhiteBalance;

        // declare_parameter
        this->declare_parameter("Height", cam_dev_->params_[CamParamType::Height]);
        this->declare_parameter("Width", cam_dev_->params_[CamParamType::Width]);
        this->declare_parameter("AutoExposure", cam_dev_->params_[CamParamType::AutoExposure]);
        this->declare_parameter("Exposure", cam_dev_->params_[CamParamType::Exposure]);
        this->declare_parameter("AutoWhiteBalance", cam_dev_->params_[CamParamType::AutoWhiteBalance]);
        this->declare_parameter("RGain", cam_dev_->params_[CamParamType::RGain]);
        this->declare_parameter("GGain", cam_dev_->params_[CamParamType::GGain]);
        this->declare_parameter("BGain", cam_dev_->params_[CamParamType::BGain]);
        this->declare_parameter("Gamma", cam_dev_->params_[CamParamType::Gamma]);
        this->declare_parameter("Fps", cam_dev_->params_[CamParamType::Fps]);

        // get parameter
        this->get_parameter("Height", Height);
        this->get_parameter("Width", Width);
        this->get_parameter("AutoExposure", AutoExposure);
        this->get_parameter("Exposure", Exposure);
        this->get_parameter("AutoWhiteBalance", AutoWhiteBalance);
        this->get_parameter("RGain", RGain);
        this->get_parameter("GGain", GGain);
        this->get_parameter("BGain", BGain);
        this->get_parameter("Gamma", Gamma);
        this->get_parameter("Fps", Fps);


        cv::FileStorage fs("./src/rmos_cam/config/daheng_camera.xml", cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Open daheng_camera.xml fail!");
            exit(0);
        }
        fs["height"] >> Height;
        fs["width"] >> Width;
        fs["exposure"] >> Exposure;
        fs["gain"] >> Gamma;





        // set paramter
        cam_dev_->set_parameter(CamParamType::Height, Height);
        cam_dev_->set_parameter(CamParamType::Width, Width);
        cam_dev_->set_parameter(CamParamType::AutoExposure, AutoExposure);
        cam_dev_->set_parameter(CamParamType::Exposure, Exposure);
        cam_dev_->set_parameter(CamParamType::AutoWhiteBalance, AutoWhiteBalance);
        cam_dev_->set_parameter(CamParamType::RGain, RGain);
        cam_dev_->set_parameter(CamParamType::GGain, GGain);
        cam_dev_->set_parameter(CamParamType::BGain, BGain);
        cam_dev_->set_parameter(CamParamType::Gamma, Gamma);
        cam_dev_->set_parameter(CamParamType::Fps, Fps);

        cam_dev_->open();

        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);   

        capture_thread_ = std::thread{[this]() -> void
                                      {
                                          while (rclcpp::ok())
                                          {
                                              if (!cam_dev_->is_open())
                                              {
                                                  exit(0);
                                              }
                                              //sensor_msgs::msg::Image image_msg_;

                                              if (cam_dev_->grab_image(image_msg_))
                                              {
                                                  image_msg_.header.frame_id = "camera";

                                                  img_pub_->publish(image_msg_);
                                                  //std::this_thread::sleep_for(std::chrono::milliseconds(3));
                                                  //RCLCPP_INFO(this->get_logger(), "Publish Image");

                                              }
                                              else
                                              {
                                                  std::cout << cam_dev_->error_message() << std::endl;
                                                  exit(0);
                                              }
                                          }
                                      }};
    }

    DahengCamNode::~DahengCamNode()
    {
        if (capture_thread_.joinable())
        {
            capture_thread_.join();
        }
        cam_dev_->close();
        RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
    }
} // namespace rmos_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::DahengCamNode)