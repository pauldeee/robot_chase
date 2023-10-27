#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl/publisher.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <tf2/LinearMath/Matrix3x3.h>

class RobotChase : public rclcpp::Node {

public:
  RobotChase() : Node("robot_chase") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&RobotChase::timer_callback, this));
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  geometry_msgs::msg::Twist twist_;

  const double KP_DISTANCE = 0.5;
  const double KP_YAW = 0.9;
  const double DISTANCE_THRESHOLD = 0.5;
  const double LINEAR_LIMIT = 1.0;

  double error_distance_ = 0, error_yaw_ = 0;

  void timer_callback() {

    geometry_msgs::msg::TransformStamped tf_r1_r2;

    try {
      tf_r1_r2 = tf_buffer_->lookupTransform(
          "rick_base_link", "morty_base_link", rclcpp::Time(0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
      return;
    }

    // get distance
    error_distance_ = sqrt(pow(tf_r1_r2.transform.translation.x, 2) +
                           pow(tf_r1_r2.transform.translation.y, 2));

    // move forward
    if (error_distance_ >= DISTANCE_THRESHOLD)
      twist_.linear.x = std::min((KP_DISTANCE * error_distance_), LINEAR_LIMIT);
    else {
      twist_.linear.x = 0.0; // stop
      twist_.angular.z = 0.0;
      twist_pub_->publish(twist_);
      return;
    }
    // get yaw for facing robot1
    error_yaw_ = std::atan2(tf_r1_r2.transform.translation.y,
                            tf_r1_r2.transform.translation.x);

    twist_.angular.z = KP_YAW * error_yaw_;

    // publish twist
    twist_pub_->publish(twist_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotChase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
