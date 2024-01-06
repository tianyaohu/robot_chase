#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

using namespace std;

class RobotChaser : public rclcpp::Node {
public:
  RobotChaser(const std::string chaser_robot_name,
              const std::string target_robot_name)
      : Node("tf2_listener_node"), chaser_robot_name(chaser_robot_name),
        target_robot_name(target_robot_name) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // Create a timer to periodically check for transforms
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&RobotChaser::timer_callback, this));

    // init command vel pub
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        chaser_robot_name + "/cmd_vel", 10);
  }

private:
  void timer_callback() {
    float trans_x, trans_y, trans_z;

    try {
      //   Look up the transform from "base_link" to "target_frame"
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform(chaser_robot_name + "/base_link",
                                      target_robot_name + "/base_link",
                                      tf2::TimePointZero);

      // translations
      trans_x = transform_stamped.transform.translation.x;
      trans_y = transform_stamped.transform.translation.y;
      trans_z = transform_stamped.transform.translation.z;

      // Print the transform
      RCLCPP_INFO(get_logger(), "Received transform: [%f, %f, %f]",
                  transform_stamped.transform.translation.x,
                  transform_stamped.transform.translation.y,
                  transform_stamped.transform.translation.z);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Transform exception: %s", ex.what());
    }

    // calculate distance error
    float error_distance =
        sqrt(trans_x * trans_x + trans_y * trans_y + trans_z * trans_z);

    cout << "distance_error is " << error_distance << endl;

    double error_yaw = atan2(trans_y, trans_x);

    // k factor for error
    double kp_distance = 0.5;
    double kp_yaw = 5;

    // max speed for linear x and angular z
    double max_linear_x = 0.6;
    double max_angular_z = 1;

    // init a message to move chaser robot
    geometry_msgs::msg::Twist move;
    move.angular.z = min(max_angular_z, kp_yaw * error_yaw);
    // minimum distance
    if (error_distance > 0.38) { // min is 0.38 VERY CLOSE
      move.linear.x = min(max_linear_x, kp_distance * error_distance);
    } else {
      move.linear.x = 0;
    }

    cout << "angular z is " << move.angular.z << endl;
    cout << "linear x is " << move.linear.x << endl;

    vel_pub_->publish(move);
  }

  std::string chaser_robot_name;
  std::string target_robot_name;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // init command vel pub
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotChaser>("rick", "morty");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
