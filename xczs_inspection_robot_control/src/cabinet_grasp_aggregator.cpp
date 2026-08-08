// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace xczs_inspection_robot_control
{

class CabinetGraspAggregator final : public rclcpp::Node
{
public:
  CabinetGraspAggregator()
  : Node("xczs_cabinet_grasp_aggregator")
  {
    const auto input_topics = declare_parameter<std::vector<std::string>>(
      "input_topics", std::vector<std::string>{});
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/xczs/cabinet/grasp_active");
    if (input_topics.empty() || output_topic.empty()) {
      throw std::invalid_argument(
              "input_topics and output_topic must not be empty.");
    }
    const std::unordered_set<std::string> unique_topics(
      input_topics.begin(), input_topics.end());
    if (unique_topics.size() != input_topics.size() ||
      unique_topics.count("") != 0U)
    {
      throw std::invalid_argument(
              "input_topics must contain unique, non-empty topic names.");
    }

    publisher_ = create_publisher<std_msgs::msg::Bool>(
      output_topic, rclcpp::QoS(1).reliable().transient_local());
    for (const auto & topic : input_topics) {
      states_.emplace(topic, false);
      subscriptions_.push_back(
        create_subscription<std_msgs::msg::Bool>(
          topic, rclcpp::QoS(1).reliable().transient_local(),
          [this, topic](const std_msgs::msg::Bool::SharedPtr message) {
            receive(topic, message->data);
          }));
    }
    publish(false);
    RCLCPP_INFO(
      get_logger(), "Aggregating grasp state from %zu cabinet instances.",
      input_topics.size());
  }

private:
  void receive(const std::string & topic, bool active)
  {
    bool any_active = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      states_[topic] = active;
      for (const auto & entry : states_) {
        any_active = any_active || entry.second;
      }
      if (published_state_ == any_active) {
        return;
      }
      published_state_ = any_active;
    }
    publish(any_active);
  }

  void publish(bool active)
  {
    std_msgs::msg::Bool message;
    message.data = active;
    publisher_->publish(message);
  }

  std::mutex mutex_;
  std::unordered_map<std::string, bool> states_;
  bool published_state_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
  subscriptions_;
};

}  // namespace xczs_inspection_robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<
        xczs_inspection_robot_control::CabinetGraspAggregator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("xczs_cabinet_grasp_aggregator"), "%s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
