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

#ifndef AWVIZ_COMMON__TRANSFORMATION__TRANSFORM_LOGGER_HPP_
#define AWVIZ_COMMON__TRANSFORMATION__TRANSFORM_LOGGER_HPP_

#include "awviz_common/transformation/tf_tree.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <tf2_ros/buffer.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace awviz_common
{
/**
 * @brief Interface for logging transforms to a recording stream.
 */
class ITransformLogger
{
public:
  virtual ~ITransformLogger() = default;

  /**
   * @brief Log a transform for the given frame.
   *
   * @param frame Frame metadata.
   */
  virtual void log_transform(const TfFrame & frame) = 0;
};

/**
 * @brief Default transform logger that uses TF buffer and entity paths.
 */
class TransformLogger final : public ITransformLogger
{
public:
  TransformLogger(
    rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> stream,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<std::unordered_map<std::string, std::string>> entities,
    std::shared_ptr<std::mutex> entities_mtx);

  void log_transform(const TfFrame & frame) override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rerun::RecordingStream> stream_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<std::unordered_map<std::string, std::string>> entities_;
  std::unordered_map<std::string, double> last_log_stamps_;
  std::shared_ptr<std::mutex> entities_mtx_;
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__TRANSFORMATION__TRANSFORM_LOGGER_HPP_
