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

#ifndef AWVIZ_COMMON__TRANSFORMATION_MANAGER_HPP_
#define AWVIZ_COMMON__TRANSFORMATION_MANAGER_HPP_

#include "awviz_common/transformation/tf_tree.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace awviz_common
{
/**
 * @brief A class to manage transform frames.
 */
class TransformationManager
{
public:
  /**
   * @brief Construct a new object.
   *
   * @param node Node shared pointer.
   * @param stream RecordingStream shared pointer.
   */
  TransformationManager(
    rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> stream);

  /**
   * @brief Return the entity paths.
   *
   * @return Shared pointer of the entity paths map.
   */
  const std::shared_ptr<std::unordered_map<std::string, std::string>> entities() const
  {
    return entities_;
  }

private:
  /**
   * @brief Callback to subscribe transformation frames.
   * @note This callback is invoked every 100 [ms].
   */
  void timer_callback();

  /**
   * @brief Update TF tree with subscribed a transformation frame.
   */
  void update_tree();

  /**
   * @brief Update the entity path of the corresponding frame.
   *
   * @param frame `TfFrame` object.
   */
  void update_entity(const TfFrame & frame);

  /**
   * @brief Log a TfFrame object in the recording stream.
   *
   * @param frame `TfFrame` object.
   */
  void log_transform(const TfFrame & frame);

private:
  rclcpp::Node::SharedPtr node_;                             //!< Node shared pointer.
  std::shared_ptr<rerun::RecordingStream> stream_;           //!< RecordingStream shared pointer.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  //!< TransformListener shared pointer.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;               //!< Buffer shared pointer.
  rclcpp::TimerBase::SharedPtr timer_;                       //!< TimerBase shared pointer.
  std::unique_ptr<TfTree> tf_tree_;                          //!< TfTree unique pointer.
  std::shared_ptr<std::unordered_map<std::string, std::string>>
    entities_;  //!< Map stores a entity path of a corresponding frame ID.
  std::unordered_map<std::string, double> last_log_stamps_;  //!< Map stores last log timestamps.
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__TRANSFORMATION_MANAGER_HPP_
