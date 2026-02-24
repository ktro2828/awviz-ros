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

#include "awviz_common/transformation/transform_logger.hpp"

#include "awviz_common/transformation/tf_tree.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <geometry_msgs/msg/transform.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

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
TransformLogger::TransformLogger(
  rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> stream,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<std::unordered_map<std::string, std::string>> entities,
  std::shared_ptr<std::mutex> entities_mtx)
: node_(std::move(node)),
  stream_(std::move(stream)),
  tf_buffer_(std::move(tf_buffer)),
  entities_(std::move(entities)),
  entities_mtx_(std::move(entities_mtx))
{
}

void TransformLogger::log_transform(const TfFrame & frame)
{
  std::lock_guard<std::mutex> lock(*entities_mtx_);

  if (!entities_ || frame.is_root() || entities_->count(frame.id()) == 0) {
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
      node_->get_logger(), "Failed to lookup transform: [source] " << frame.id() << ", [target] "
                                                                   << frame.parent()
                                                                   << ", error: " << ex.what());
  }
}
}  // namespace awviz_common
