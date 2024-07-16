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

#include "awviz/geometry.hpp"
#include "awviz/rerun_ros_interface.hpp"
#include "awviz/topic_option.hpp"
#include "rclcpp/subscription.hpp"
#include "tf2/exceptions.h"

#include <chrono>

namespace awviz
{
using std::chrono_literals::operator""ms;
RerunLoggerNode::RerunLoggerNode(const rclcpp::NodeOptions & node_options)
: Node("rerun_logger_node", node_options), stream_("rerun_logger_node")
{
  stream_.spawn().exit_on_failure();

  topic_options_ = TopicOption::fromRosParam(this);

  parallel_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  callback_timer_ = create_wall_timer(
    100ms, [&]() -> void { createSubscriptions(); }, parallel_callback_group_);
}

void RerunLoggerNode::createSubscriptions()
{
  // TODO(ktro2828): use rclcpp::Node::get_topic_names_and_types()
  for (const auto & option : topic_options_) {
    if (topic_to_subscription_.find(option.topic()) != topic_to_subscription_.end()) {
      continue;
    }

    if (option.type() == MsgType::PointCloud) {
      topic_to_subscription_[option.topic()] = createPointCloudSubscription(option);
    } else if (option.type() == MsgType::CameraInfo) {
      topic_to_subscription_[option.topic()] = createCameraInfoSubscription(option);
    } else if (option.type() == MsgType::Image) {
      topic_to_subscription_[option.topic()] = createImageSubscription(option);
    } else if (option.type() == MsgType::CompressedImage) {
      topic_to_subscription_[option.topic()] = createCompressedImageSubscription(option);
    } else if (option.type() == MsgType::DetectedObjects) {
      topic_to_subscription_[option.topic()] = createDetectedObjectsSubscription(option);
    } else if (option.type() == MsgType::TrackedObjects) {
      topic_to_subscription_[option.topic()] = createTrackedObjectsSubscription(option);
    } else {
      topic_to_subscription_[option.topic()] = nullptr;
      RCLCPP_WARN_STREAM(this->get_logger(), "Unknown msg type of topic: " << option.topic());
    }
  }
}

bool RerunLoggerNode::logMapTransform(
  const std_msgs::msg::Header & header, const TopicOption & option) const
{
  try {
    auto transform = tf_buffer_->lookupTransform("map", header.frame_id, header.stamp, 100ms);
    awviz::logTransform(stream_, option.entity(), transform);
    return true;
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), e.what());
    return false;
  }
}

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
RerunLoggerNode::createPointCloudSubscription(const TopicOption & option)
{
  return this->create_subscription<sensor_msgs::msg::PointCloud2>(
    option.topic(), 1000, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      if (!isMapFrameId(msg->header) && !logMapTransform(msg->header, option)) {
        return;
      }
      awviz::logPointCloud(stream_, option.entity(), msg);
    });
}

rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
RerunLoggerNode::createCameraInfoSubscription(const TopicOption & option)
{
  return this->create_subscription<sensor_msgs::msg::CameraInfo>(
    option.topic(), 1000, [&](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      awviz::logCameraInfo(stream_, option.entity(), msg);
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

rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
RerunLoggerNode::createCompressedImageSubscription(const TopicOption & option)
{
  return this->create_subscription<sensor_msgs::msg::CompressedImage>(
    option.topic(), 1000, [&](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
      awviz::logCompressedImage(stream_, option.entity(), msg);
    });
}

rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
RerunLoggerNode::createDetectedObjectsSubscription(const TopicOption & option)
{
  return this->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    option.topic(), 1000, [&](const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg) {
      if (!isMapFrameId(msg->header) && !logMapTransform(msg->header, option)) {
        return;
      }
      awviz::logDetectedObjects(stream_, option.entity(), msg);
    });
}

rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr
RerunLoggerNode::createTrackedObjectsSubscription(const TopicOption & option)
{
  return this->create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
    option.topic(), 1000, [&](const autoware_perception_msgs::msg::TrackedObjects::SharedPtr msg) {
      if (!isMapFrameId(msg->header) && !logMapTransform(msg->header, option)) {
        return;
      }
      awviz::logTrackedObjects(stream_, option.entity(), msg);
    });
}
}  // namespace awviz

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(awviz::RerunLoggerNode);
