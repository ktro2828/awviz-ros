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

#include "awviz_common/transformation/transformation_manager.hpp"

#include <geometry_msgs/msg/transform.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

namespace
{
/**
 * @brief Convert `geometry_msgs::msg::Transform` to `rerun::Transform3D`.
 *
 * @param transform ROS message transform.
 * @return Converted instance.
 */
rerun::Transform3D from_msg(const geometry_msgs::msg::Transform & transform)
{
  return rerun::Transform3D::from_translation({static_cast<float>(transform.translation.x),
                                               static_cast<float>(transform.translation.y),
                                               static_cast<float>(transform.translation.z)})
    .with_quaternion(rerun::Quaternion::from_wxyz(
      transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));
}
}  // namespace

namespace awviz_common
{
TransformationManager::TransformationManager(
  rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> stream)
: node_(std::move(node)),
  stream_(std::move(stream)),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
  tf_tree_(std::make_unique<TfTree>()),
  entities_(std::make_shared<std::unordered_map<std::string, std::string>>())
{
  // NOTE: Invoke callback every 100 [ms]
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&TransformationManager::timer_callback, this));
}

void TransformationManager::timer_callback()
{
  // 1. Parse all frames and update TF tree
  update_tree();

  // 2. Update entities
  for (const auto & [_, frame] : tf_tree_->get_frames()) {
    update_entity(frame);
  }

  // 3. Log transformations in the recording stream.
  for (const auto & [_, frame] : tf_tree_->get_frames()) {
    log_transform(frame);
  }
}

void TransformationManager::update_tree()
{
  auto yaml = tf_buffer_->allFramesAsYAML();
  try {
    YAML::Node frames = YAML::Load(yaml);
    for (YAML::const_iterator itr = frames.begin(); itr != frames.end(); ++itr) {
      std::lock_guard lock(entities_mtx_);

      const auto id = itr->first.as<std::string>();
      const auto parent = itr->second["parent"].as<std::string>();

      const TfFrame frame(id, parent);

      tf_tree_->emplace(frame);
    }
  } catch (const YAML::Exception & ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to parse YAML: " << ex.what());
  }
}

void TransformationManager::update_entity(const TfFrame & frame)
{
  std::scoped_lock lock(tf_tree_mtx_, entities_mtx_);
  if (!tf_tree_->can_link_to(frame, TF_ROOT)) {
    return;
  }
  const auto entity = tf_tree_->entity_path(frame);
  entities_->emplace(frame.id(), entity);
}

void TransformationManager::log_transform(const TfFrame & frame)
{
  std::lock_guard lock(entities_mtx_);
  // Root frame is skipped to log transformation
  if (frame.is_root() || entities_->count(frame.id()) == 0) {
    return;
  }

  try {
    const auto tf_stamped =
      tf_buffer_->lookupTransform(frame.parent(), frame.id(), tf2::TimePointZero);

    const auto timestamp =
      rclcpp::Time(tf_stamped.header.stamp.sec, tf_stamped.header.stamp.nanosec).seconds();

    if (last_log_stamps_.count(frame.id()) > 0 && last_log_stamps_.at(frame.id()) == timestamp) {
      return;
    }

    const auto & entity_path = entities_->at(frame.id());

    if (frame.is_static()) {
      stream_->log_static(entity_path, from_msg(tf_stamped.transform));
    } else {
      stream_->set_time_seconds("timestamp", timestamp);
      stream_->log(entity_path, from_msg(tf_stamped.transform));
    }
    last_log_stamps_[frame.id()] = timestamp;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Fail to lookup transformation: [source] " << frame.id() << ", [target] " << frame.parent());
  }
}

}  // namespace awviz_common
