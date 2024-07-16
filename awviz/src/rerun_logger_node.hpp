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

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
   * @brief Log Transform3D for map frame id to rerun stream.
   *
   * @param header
   * @param option
   * @return Return true, if logging is succeeded.
   */
  bool logMapTransform(const std_msgs::msg::Header & header, const TopicOption & option) const;

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

  /**
   * @brief Create subscriber for DetectedObjects msg.
   * @param option Topic option.
   */
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
  createDetectedObjectsSubscription(const TopicOption & option);

  /**
   * @brief Create subscriber for TrackedObjects msg.
   * @param option Topic option.
   */
  rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr
  createTrackedObjectsSubscription(const TopicOption & option);

private:
  const rerun::RecordingStream stream_;

  rclcpp::CallbackGroup::SharedPtr parallel_callback_group_;
  std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> topic_to_subscription_;
  std::map<std::string, std::string> frame_id_to_entity_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr callback_timer_;
  rclcpp::TimerBase::SharedPtr update_tf_timer_;

  std::vector<TopicOption> topic_options_;
};
}  // namespace awviz

#endif  // AWVIZ_RERUN_LOGGER_NODE_HPP_
