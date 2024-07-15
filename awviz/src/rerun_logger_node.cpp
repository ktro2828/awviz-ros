// Copyright 2024 Kotaro Uetake.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rerun_logger_node.hpp"

#include "awviz/rerun_ros_interface.hpp"
#include "awviz/topic_option.hpp"
#include "rclcpp/subscription.hpp"

#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#include <chrono>

namespace awviz
{
RerunLoggerNode::RerunLoggerNode(const rclcpp::NodeOptions & node_options)
: Node("rerun_logger_node", node_options), stream_("rerun_logger_node")
{
  using std::chrono_literals::operator""ms;

  stream_.spawn().exit_on_failure();

  topic_options_ = TopicOption::fromRosParam(this);

  parallel_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  callback_timer_ = create_wall_timer(
    100ms, [&]() -> void { createSubscriptions(); }, parallel_callback_group_);
}

void RerunLoggerNode::createSubscriptions()
{
  for (const auto & option : topic_options_) {
    if (topic_to_subscription_.find(option.topic()) != topic_to_subscription_.end()) {
      continue;
    }

    if (option.type() == MsgType::PointCloud) {
      topic_to_subscription_[option.topic()] = createPointCloudSubscription(option);
    } else if (option.type() == MsgType::Image) {
      topic_to_subscription_[option.topic()] = createImageSubscription(option);
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), "Unknown msg type of topic: " << option.topic());
    }
  }
}

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
RerunLoggerNode::createPointCloudSubscription(const TopicOption & option)
{
  return this->create_subscription<sensor_msgs::msg::PointCloud2>(
    option.topic(), 1000, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      awviz::logPointCloud(stream_, option.entity(), msg);
    });
}

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr RerunLoggerNode::createImageSubscription(
  const TopicOption & option)
{
  return this->create_subscription<sensor_msgs::msg::Image>(
    option.topic(), 1000, [&](const sensor_msgs::msg::Image::SharedPtr msg) {
      awviz::logImage(stream_, option.entity(), msg);
    });
}
}  // namespace awviz

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(awviz::RerunLoggerNode);
