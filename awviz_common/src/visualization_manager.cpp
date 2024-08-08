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

#include "awviz_common/visualization_manager.hpp"

#include <chrono>
#include <memory>

namespace awviz_common
{
VisualizationManager::VisualizationManager(
  rclcpp::Node::SharedPtr node, const std::shared_ptr<rerun::RecordingStream> & stream)
: node_(node), stream_(stream)
{
  using std::chrono_literals::operator""ms;

  display_factory_ = std::make_unique<DisplayFactory>();

  parallel_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  callback_timer_ = node_->create_wall_timer(
    100ms, [&]() { createSubscriptions(); }, parallel_callback_group_);
}

void VisualizationManager::createSubscriptions()
{
  for (const auto & [topic_name, topic_types] : node_->get_topic_names_and_types()) {
    if (display_group_.find(topic_name) != display_group_.cend()) {
      continue;
    }

    const auto topic_type = topic_types.front();
  }
}

}  // namespace awviz_common