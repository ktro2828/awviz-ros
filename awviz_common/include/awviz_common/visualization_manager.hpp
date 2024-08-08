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
  VisualizationManager(
    rclcpp::Node::SharedPtr node, const std::shared_ptr<rerun::RecordingStream> & stream);

private:
  rclcpp::Node::SharedPtr node_;
  const std::shared_ptr<rerun::RecordingStream> stream_;
  std::unique_ptr<DisplayFactory> display_factory_;
  std::map<std::string, std::shared_ptr<Display>> display_group_;
  rclcpp::CallbackGroup::SharedPtr parallel_callback_group_;
  rclcpp::TimerBase::SharedPtr callback_timer_;

private:
  void createSubscriptions();
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__VISUALIZATION_MANAGER_HPP_
