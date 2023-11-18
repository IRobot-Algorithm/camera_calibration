/**
 *  订阅图像话题
 */ 
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
namespace rmos_cam
{

class CamSubNode : public rclcpp::Node{
public:
    CamSubNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("cam_sub_node", options){
        time_point_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Start camera sub node");
        img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                    this, "/image_raw", std::bind(&CamSubNode::imageCallback, this, std::placeholders::_1), "raw",
                    rmw_qos_profile_sensor_data));
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>
        ("/camera_info", rclcpp::SensorDataQoS(), std::bind(&CamSubNode::cameraInfoCallBack, this, std::placeholders::_1));
        
        img_pub_ = std::make_shared<image_transport::Publisher>(image_transport::create_publisher(
            this, "/image_test", rmw_qos_profile_default
        ));
    }
    ~CamSubNode(){
        
    }
    std::shared_ptr<image_transport::Subscriber> img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::shared_ptr<image_transport::Publisher> img_pub_;
    std::chrono::steady_clock::time_point time_point_;

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg){
        // RCLCPP_INFO(this->get_logger(), "Get Image");
        // RCLCPP_INFO(this->get_logger(), "Delay of image transform: %ld μs", (this->now() - image_msg_->header.stamp).nanoseconds() / 1000);
        // RCLCPP_INFO(this->get_logger(), "Interval: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_point_).count());
        time_point_ = std::chrono::steady_clock::now();
        auto image = cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
        cv::imshow("image", image);
        char key = cv::waitKey(1);
        if (key == 's'){
            cv::imwrite("/home/shanzoom/images/" + image_msg->header.frame_id + ".jpg", image);
            RCLCPP_INFO(this->get_logger(), "Save image");
        }

        sensor_msgs::msg::Image image_msg_trans_ = *image_msg;
        image_msg_trans_.header.stamp = this->now();
        img_pub_->publish(image_msg_trans_);
    }

    void cameraInfoCallBack(sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg_){
        RCLCPP_INFO(this->get_logger(), "Get Camera Infomation: %s", camera_info_msg_->header.frame_id.c_str());
        camera_info_sub_.reset();
    }
};    

} // namespace rm

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::CamSubNode)

