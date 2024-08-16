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

#ifndef AWVIZ_COMMON__VISUALIZATION_MANAGER_HPP_
#define AWVIZ_COMMON__VISUALIZATION_MANAGER_HPP_

#include "awviz_common/display.hpp"
#include "awviz_common/display_factory.hpp"
#include "awviz_common/transformation/transformation_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <map>
#include <memory>
#include <string>

namespace awviz_common
{
/**
 * @brief Manager class to handle visualization processes.
 */
class VisualizationManager
{
public:
  /**
   * @brief Construct a new Visualization Manager object.
   *
   * @param node Shared pointer of a node object.
   * @param stream Shared pointer of a recording stream object.
   */
  VisualizationManager(
    rclcpp::Node::SharedPtr node, const std::shared_ptr<rerun::RecordingStream> & stream);

  /**
   * @brief Destroy the VisualizationManager object with unsubscribing topics of displays.
   */
  ~VisualizationManager();

private:
  rclcpp::Node::SharedPtr node_;                          //!< Node shared pointer.
  const std::shared_ptr<rerun::RecordingStream> stream_;  //!< RecordingStream shared pointer.
  std::unique_ptr<DisplayFactory> display_factory_;       //!< Factory to load display class.
  std::map<std::string, std::shared_ptr<Display>>
    display_group_;  //!< Buffer storing created displays, which key is topic name and value is its
                     //!< shared pointer.
  std::unique_ptr<TransformationManager> tf_manager_;         //!< Transformation manager.
  rclcpp::CallbackGroup::SharedPtr parallel_callback_group_;  //!< Parallel callback group.
  rclcpp::TimerBase::SharedPtr callback_timer_;               //!< Timer callback.

private:
  /**
   * @brief Parse topics and create new ROS subscriptions every time.
   */
  void createSubscriptions();
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__VISUALIZATION_MANAGER_HPP_
