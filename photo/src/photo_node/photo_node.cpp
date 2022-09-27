
#include <mutex>
// ROS Headers
#include <rclcpp/rclcpp.hpp>
// ROS Messages
#include <cinttypes>
#include <sensor_msgs/fill_image.hpp>

// ROS Services
#include "photo_msgs/srv/capture.hpp"
#include "photo_msgs/srv/get_config.hpp"
#include "photo_msgs/srv/set_config.hpp"
// photo library headers
#include "photo/photo_camera.hpp"
#include "photo/photo_camera_list.hpp"
#include "photo/photo_image.hpp"

class PhotoNode : public rclcpp::Node {
public:
  photo_camera_list camera_list_;
  photo_camera camera_;
  photo_image image_;

  std::mutex photo_mutex_;
  rclcpp::Service<photo_msgs::srv::Capture>::SharedPtr capture_srv_;
  rclcpp::Service<photo_msgs::srv::SetConfig>::SharedPtr set_config_srv_;
  rclcpp::Service<photo_msgs::srv::GetConfig>::SharedPtr get_config_srv_;

  PhotoNode() : Node("photo_node"), camera_list_(), camera_(), image_() {

    GPContext *private_context;

    // initialize camera

    // create context
    private_context = camera_.photo_camera_create_context();

    // autodetect all cameras connected
    if (camera_list_.autodetect(private_context) == false) {
      RCLCPP_FATAL(get_logger(),
                   "photo_node: Autodetection of cameras failed.");
      gp_context_unref(private_context);
      rclcpp::shutdown();
      return;
    }

    // open camera from camera list
    if (camera_.photo_camera_open(&camera_list_, 0) == false) {
      RCLCPP_FATAL(get_logger(), "photo_node: Could not open camera %d.", 0);
      gp_context_unref(private_context);
      rclcpp::shutdown();
      return;
    }

    // ***** Start Services *****
    set_config_srv_ = create_service<photo_msgs::srv::SetConfig>(
        "set_config", std::bind(&PhotoNode::setConfig, this,
                                std::placeholders::_1, std::placeholders::_2));
    get_config_srv_ = create_service<photo_msgs::srv::GetConfig>(
        "get_config", std::bind(&PhotoNode::getConfig, this,
                                std::placeholders::_1, std::placeholders::_2));
    capture_srv_ = create_service<photo_msgs::srv::Capture>(
        "capture", std::bind(&PhotoNode::capture, this, std::placeholders::_1,
                             std::placeholders::_2));
  }

  ~PhotoNode() {
    // shutdown camera
    camera_.photo_camera_close();
  }

  bool
  setConfig(const std::shared_ptr<photo_msgs::srv::SetConfig::Request> request,
            std::shared_ptr<photo_msgs::srv::SetConfig::Response> response) {
    photo_mutex_.lock();

    bool error_code =
        camera_.photo_camera_set_config(request->param, request->value);
    photo_mutex_.unlock();
    return error_code;
  }

  bool
  getConfig(const std::shared_ptr<photo_msgs::srv::GetConfig::Request> request,
            std::shared_ptr<photo_msgs::srv::GetConfig::Response> response) {
    char *value = new char[255];
    photo_mutex_.lock();
    bool error_code = camera_.photo_camera_get_config(request->param, &value);
    if (error_code) {
      response->value = value;
    }
    photo_mutex_.unlock();
    delete[] value;
    return error_code;
  }

  bool capture(const std::shared_ptr<photo_msgs::srv::Capture::Request> request,
               std::shared_ptr<photo_msgs::srv::Capture::Response> response) {
    // capture a camera image
    photo_mutex_.lock();
    bool error_code = camera_.photo_camera_capture(&image_);
    if (error_code) {
      // fill image message
      sensor_msgs::fillImage(response->image, "rgb8", image_.getHeight(),
                             image_.getWidth(),
                             image_.getBytesPerPixel() * image_.getWidth(),
                             image_.getDataAddress());
    }
    photo_mutex_.unlock();
    return error_code;
  }
};

int main(int argc, char **argv) {
  rclcpp::executors::MultiThreadedExecutor exec;
  std::shared_ptr<PhotoNode> node(new PhotoNode());
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
