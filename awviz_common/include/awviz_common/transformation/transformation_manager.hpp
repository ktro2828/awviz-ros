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

#include "awviz_common/transformation/entity_path_resolver.hpp"
#include "awviz_common/transformation/tf_tree.hpp"
#include "awviz_common/transformation/tf_tree_updater.hpp"
#include "awviz_common/transformation/transform_logger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
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
  const std::shared_ptr<std::unordered_map<std::string, std::string>> entities()
  {
    return entity_path_resolver_->entities();
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
   * @brief Update entity paths for the current TF tree.
   */
  void update_entities();

  /**
   * @brief Log transforms for the current TF tree.
   */
  void log_transforms();

private:
  rclcpp::Node::SharedPtr node_;                              //!< Node shared pointer.
  std::shared_ptr<rerun::RecordingStream> stream_;            //!< RecordingStream shared pointer.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                //!< Buffer shared pointer.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   //!< TransformListener shared pointer.
  rclcpp::TimerBase::SharedPtr timer_;                        //!< TimerBase shared pointer.
  std::shared_ptr<TfTree> tf_tree_;                           //!< TfTree shared pointer.
  std::shared_ptr<std::mutex> tf_tree_mtx_;                   //!< Mutex for TF tree updates.
  std::shared_ptr<std::mutex> entities_mtx_;                  //!< Mutex for entity map access.
  std::unique_ptr<ITfTreeUpdater> tf_tree_updater_;           //!< TF tree updater.
  std::unique_ptr<EntityPathResolver> entity_path_resolver_;  //!< Entity path resolver.
  std::unique_ptr<ITransformLogger> transform_logger_;        //!< Transform logger.
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__TRANSFORMATION_MANAGER_HPP_
