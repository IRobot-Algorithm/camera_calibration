/**
 *  测试多个节点订阅延迟
 */
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <sstream>
namespace rmos_cam
{

class CamSubSubNode : public rclcpp::Node{
public:
    CamSubSubNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("cam_sub_node", options){
        time_point_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Start camera sub node");
        img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                    this, "/image_test", std::bind(&CamSubSubNode::imageCallback, this, std::placeholders::_1), "raw",
                    rmw_qos_profile_sensor_data));

    }
    ~CamSubSubNode(){
        
    }
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    std::shared_ptr<image_transport::Subscriber> img_sub_;
    std::chrono::steady_clock::time_point time_point_;

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg_){
        RCLCPP_INFO(this->get_logger(), "Get Image");
        RCLCPP_INFO(this->get_logger(), "Delay of image transform: %ld μs", (this->now() - image_msg_->header.stamp).nanoseconds() / 1000);
        auto image = cv_bridge::toCvShare(image_msg_, "bgr8")->image;
        RCLCPP_INFO(this->get_logger(), "Interval: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_point_).count());
        time_point_ = std::chrono::steady_clock::now();
        cv::imshow("image_test", image);
        cv::waitKey(1);

    }
};    

} // namespace rm

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::CamSubSubNode)