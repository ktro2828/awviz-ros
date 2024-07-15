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

#ifndef AWVIZ_RERUN_LOGGER_NODE_HPP_
#define AWVIZ_RERUN_LOGGER_NODE_HPP_

#include "awviz/topic_option.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace awviz
{
class RerunLoggerNode : public rclcpp::Node
{
public:
  explicit RerunLoggerNode(const rclcpp::NodeOptions & node_options);

private:
  /**
   * @brief Create subscribers.
   */
  void createSubscriptions();

  /**
   * @brief Create a subscriber for PointCloud2 msg.
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr createPointCloudSubscription(
    const TopicOption & option);

  /**
   * @brief Craete subscriber for Image msg.
   * @param option Topic option.
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr createImageSubscription(
    const TopicOption & option);

  /**
   * @brief Create subscriber for CompressedImage msg.
   * @param option Topic option.
   */
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
  createCompressedImageSubscription(const TopicOption & option);

private:
  const rerun::RecordingStream stream_;

  rclcpp::CallbackGroup::SharedPtr parallel_callback_group_;
  std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> topic_to_subscription_;
  std::map<std::string, std::string> frame_id_to_entity_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr callback_timer_;
  rclcpp::TimerBase::SharedPtr update_tf_timer_;

  std::vector<TopicOption> topic_options_;
};
}  // namespace awviz

#endif  // AWVIZ_RERUN_LOGGER_NODE_HPP_
