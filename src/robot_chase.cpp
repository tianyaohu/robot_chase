#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class RobotChaser : public rclcpp::Node {
public:
  RobotChaser() : Node("tf2_listener_node") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // Create a timer to periodically check for transforms
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&RobotChaser::timer_callback, this));

    // init command vel pub
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void timer_callback() {
    try {
      // Look up the transform from "base_link" to "target_frame"
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform("rick/base_link", "morty/base_link",
                                      tf2::TimePoint());

      // Print the transform
      RCLCPP_INFO(get_logger(), "Received transform: [%f, %f, %f]",
                  transform_stamped.transform.translation.x,
                  transform_stamped.transform.translation.y,
                  transform_stamped.transform.translation.z);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Transform exception: %s", ex.what());
    }

  }

  std::string start_frame;
  std::string dest_frame;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // init command vel pub
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotChaser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
