// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace xczs_inspection_robot_control
{

namespace
{

using PoseWithCovarianceStamped =
  geometry_msgs::msg::PoseWithCovarianceStamped;

constexpr char kStaticSource[] = "static";
constexpr char kTopicSource[] = "topic";
constexpr char kValidityTopic[] = "/xczs/cabinet/pose_valid";

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool finite_covariance(const std::array<double, 36> & covariance)
{
  return std::all_of(
    covariance.cbegin(), covariance.cend(),
    [](double value) {return std::isfinite(value);});
}

double quaternion_norm(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
}

void normalize_quaternion(geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = quaternion_norm(quaternion);
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
}

double translation_distance(
  const geometry_msgs::msg::Pose & first,
  const geometry_msgs::msg::Pose & second)
{
  return std::hypot(
    std::hypot(
      first.position.x - second.position.x,
      first.position.y - second.position.y),
    first.position.z - second.position.z);
}

double rotation_distance(
  const geometry_msgs::msg::Quaternion & first,
  const geometry_msgs::msg::Quaternion & second)
{
  const double dot = std::abs(
    first.x * second.x + first.y * second.y +
    first.z * second.z + first.w * second.w);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

}  // namespace

class CabinetPoseAuthority final : public rclcpp::Node
{
public:
  CabinetPoseAuthority()
  : Node("xczs_cabinet_pose_authority")
  {
    pose_source_ = declare_parameter<std::string>(
      "pose_source", kStaticSource);
    parent_frame_ = declare_parameter<std::string>(
      "parent_frame", "odom");
    cabinet_frame_ = declare_parameter<std::string>(
      "cabinet_frame", "control_cabinet_frame");
    measurement_topic_ = declare_parameter<std::string>(
      "measurement_topic", "/xczs/cabinet/pose_measurement");

    measurement_timeout_ = positive_parameter(
      "limits.measurement_timeout_sec", 1.0);
    transform_timeout_ = positive_parameter(
      "limits.transform_timeout_sec", 0.20);
    future_tolerance_ = nonnegative_parameter(
      "limits.future_tolerance_sec", 0.10);
    max_translation_variance_ = nonnegative_parameter(
      "limits.max_translation_variance", 4.0e-4);
    max_rotation_variance_ = nonnegative_parameter(
      "limits.max_rotation_variance", 7.6154e-3);
    max_translation_jump_ = positive_parameter(
      "limits.max_translation_jump_m", 0.50);
    max_rotation_jump_ = positive_parameter(
      "limits.max_rotation_jump_rad", 0.78539816339);

    validate_frame(parent_frame_, "parent_frame");
    validate_frame(cabinet_frame_, "cabinet_frame");
    if (parent_frame_ == cabinet_frame_) {
      throw std::invalid_argument(
              "Parameters 'parent_frame' and 'cabinet_frame' must differ.");
    }
    if (pose_source_ != kStaticSource && pose_source_ != kTopicSource) {
      throw std::invalid_argument(
              "Parameter 'pose_source' must be either 'static' or 'topic'.");
    }

    validity_publisher_ = create_publisher<std_msgs::msg::Bool>(
      kValidityTopic,
      rclcpp::QoS(1).reliable().transient_local());

    if (pose_source_ == kStaticSource) {
      configure_static_source();
    } else {
      configure_topic_source();
    }
  }

private:
  double positive_parameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
              "Parameter '" + name + "' must be finite and positive.");
    }
    return value;
  }

  double nonnegative_parameter(
    const std::string & name,
    double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(
              "Parameter '" + name +
              "' must be finite and nonnegative.");
    }
    return value;
  }

  static void validate_frame(
    const std::string & frame,
    const std::string & parameter_name)
  {
    if (frame.empty() || frame.front() == '/') {
      throw std::invalid_argument(
              "Parameter '" + parameter_name +
              "' must be a nonempty TF frame without a leading '/'.");
    }
  }

  void configure_static_source()
  {
    const double x = declare_parameter<double>("static_pose.x", 2.0);
    const double y = declare_parameter<double>("static_pose.y", 0.33);
    const double z = declare_parameter<double>("static_pose.z", 0.0);
    const double roll = declare_parameter<double>(
      "static_pose.roll", 1.57079632679);
    const double pitch = declare_parameter<double>("static_pose.pitch", 0.0);
    const double yaw = declare_parameter<double>(
      "static_pose.yaw", -1.57079632679);
    const std::array<double, 6> values{x, y, z, roll, pitch, yaw};
    if (!std::all_of(
        values.cbegin(), values.cend(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "All 'static_pose.*' parameters must be finite.");
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = cabinet_frame_;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    tf2::Quaternion rotation;
    rotation.setRPY(roll, pitch, yaw);
    rotation.normalize();
    transform.transform.rotation = tf2::toMsg(rotation);

    static_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    static_broadcaster_->sendTransform(transform);
    publish_validity(true);
    RCLCPP_INFO(
      get_logger(),
      "Static cabinet pose authority publishes %s -> %s from parameters.",
      parent_frame_.c_str(), cabinet_frame_.c_str());
  }

  void configure_topic_source()
  {
    if (measurement_topic_.empty() || measurement_topic_.front() != '/') {
      throw std::invalid_argument(
              "Parameter 'measurement_topic' must be an absolute topic.");
    }

    transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*transform_buffer_);
    dynamic_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(this);
    measurement_subscription_ = create_subscription<PoseWithCovarianceStamped>(
      measurement_topic_, rclcpp::SensorDataQoS(),
      [this](const PoseWithCovarianceStamped::SharedPtr message) {
        receive_measurement(*message);
      });
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {check_measurement_timeout();});

    publish_validity(false);
    RCLCPP_INFO(
      get_logger(),
      "Topic cabinet pose authority waits on %s and publishes dynamic TF "
      "%s -> %s; static fallback is disabled.",
      measurement_topic_.c_str(), parent_frame_.c_str(),
      cabinet_frame_.c_str());
  }

  void receive_measurement(const PoseWithCovarianceStamped & message)
  {
    if (message.header.frame_id.empty() ||
      message.header.frame_id.front() == '/')
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected cabinet pose with an empty frame or leading '/'.");
      return;
    }
    if (!finite_pose(message.pose.pose) ||
      !finite_covariance(message.pose.covariance))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected non-finite cabinet pose or covariance.");
      return;
    }
    const double input_quaternion_norm = quaternion_norm(
      message.pose.pose.orientation);
    if (!std::isfinite(input_quaternion_norm) ||
      input_quaternion_norm < 1.0e-6)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected cabinet pose with an invalid quaternion.");
      return;
    }

    const rclcpp::Time received_at = now();
    rclcpp::Time measurement_stamp(message.header.stamp, get_clock()->get_clock_type());
    const bool has_measurement_stamp =
      message.header.stamp.sec != 0 || message.header.stamp.nanosec != 0U;
    if (has_measurement_stamp) {
      const double age = (received_at - measurement_stamp).seconds();
      if (age > measurement_timeout_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejected cabinet pose aged %.3f s (limit %.3f s).",
          age, measurement_timeout_);
        return;
      }
      if (age < -future_tolerance_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejected cabinet pose %.3f s in the future (limit %.3f s).",
          -age, future_tolerance_);
        return;
      }
    } else {
      measurement_stamp = received_at;
    }

    PoseWithCovarianceStamped transformed;
    try {
      if (message.header.frame_id == parent_frame_) {
        transformed = message;
      } else {
        transformed = transform_buffer_->transform(
          message, parent_frame_,
          tf2::durationFromSec(transform_timeout_));
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected cabinet pose because %s -> %s is unavailable: %s",
        message.header.frame_id.c_str(), parent_frame_.c_str(), error.what());
      return;
    }
    transformed.header.frame_id = parent_frame_;
    transformed.header.stamp = measurement_stamp;
    const double transformed_quaternion_norm = quaternion_norm(
      transformed.pose.pose.orientation);
    if (!finite_pose(transformed.pose.pose) ||
      !finite_covariance(transformed.pose.covariance) ||
      !std::isfinite(transformed_quaternion_norm) ||
      transformed_quaternion_norm < 1.0e-6)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected cabinet pose because its transformed value is invalid.");
      return;
    }
    normalize_quaternion(transformed.pose.pose.orientation);

    if (!acceptable_covariance(transformed.pose.covariance)) {
      return;
    }

    bool was_valid = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      was_valid = pose_valid_;
      if (pose_valid_ && has_accepted_pose_) {
        const double translation_jump = translation_distance(
          accepted_pose_, transformed.pose.pose);
        const double rotation_jump = rotation_distance(
          accepted_pose_.orientation,
          transformed.pose.pose.orientation);
        if (translation_jump > max_translation_jump_ ||
          rotation_jump > max_rotation_jump_)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rejected cabinet pose jump of %.3f m / %.3f rad "
            "(limits %.3f m / %.3f rad).",
            translation_jump, rotation_jump,
            max_translation_jump_, max_rotation_jump_);
          return;
        }
      }
      accepted_pose_ = transformed.pose.pose;
      has_accepted_pose_ = true;
      pose_valid_ = true;
      last_accepted_at_ = std::chrono::steady_clock::now();
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = transformed.header;
    transform.child_frame_id = cabinet_frame_;
    transform.transform.translation.x = transformed.pose.pose.position.x;
    transform.transform.translation.y = transformed.pose.pose.position.y;
    transform.transform.translation.z = transformed.pose.pose.position.z;
    transform.transform.rotation = transformed.pose.pose.orientation;
    dynamic_broadcaster_->sendTransform(transform);
    if (!was_valid) {
      publish_validity(true);
      RCLCPP_INFO(
        get_logger(),
        "Accepted cabinet pose and activated dynamic TF %s -> %s.",
        parent_frame_.c_str(), cabinet_frame_.c_str());
    }
  }

  bool acceptable_covariance(const std::array<double, 36> & covariance)
  {
    constexpr std::array<std::size_t, 3> translation_diagonal{0U, 7U, 14U};
    constexpr std::array<std::size_t, 3> rotation_diagonal{21U, 28U, 35U};
    double translation_variance = 0.0;
    double rotation_variance = 0.0;
    for (const auto index : translation_diagonal) {
      if (covariance[index] < 0.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejected cabinet pose with a negative translation variance.");
        return false;
      }
      translation_variance = std::max(
        translation_variance, covariance[index]);
    }
    for (const auto index : rotation_diagonal) {
      if (covariance[index] < 0.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejected cabinet pose with a negative rotation variance.");
        return false;
      }
      rotation_variance = std::max(rotation_variance, covariance[index]);
    }
    if (translation_variance > max_translation_variance_ ||
      rotation_variance > max_rotation_variance_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected cabinet pose covariance %.6g m^2 / %.6g rad^2 "
        "(limits %.6g / %.6g).",
        translation_variance, rotation_variance,
        max_translation_variance_, max_rotation_variance_);
      return false;
    }
    return true;
  }

  void check_measurement_timeout()
  {
    bool expired = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (pose_valid_ && has_accepted_pose_) {
        const double age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_accepted_at_).count();
        if (age > measurement_timeout_) {
          pose_valid_ = false;
          expired = true;
        }
      }
    }
    if (expired) {
      publish_validity(false);
      RCLCPP_WARN(
        get_logger(),
        "Cabinet pose expired after %.3f s without an accepted measurement; "
        "dynamic TF publication remains stopped until reacquisition.",
        measurement_timeout_);
    }
  }

  void publish_validity(bool valid)
  {
    std_msgs::msg::Bool message;
    message.data = valid;
    validity_publisher_->publish(message);
  }

  std::string pose_source_;
  std::string parent_frame_;
  std::string cabinet_frame_;
  std::string measurement_topic_;
  double measurement_timeout_{1.0};
  double transform_timeout_{0.20};
  double future_tolerance_{0.10};
  double max_translation_variance_{4.0e-4};
  double max_rotation_variance_{7.6154e-3};
  double max_translation_jump_{0.50};
  double max_rotation_jump_{0.78539816339};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validity_publisher_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr
    measurement_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<tf2_ros::Buffer> transform_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::mutex state_mutex_;
  geometry_msgs::msg::Pose accepted_pose_;
  std::chrono::steady_clock::time_point last_accepted_at_{};
  bool has_accepted_pose_{false};
  bool pose_valid_{false};
};

}  // namespace xczs_inspection_robot_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<
        xczs_inspection_robot_control::CabinetPoseAuthority>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "Cabinet pose authority failed: %s\n", error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
