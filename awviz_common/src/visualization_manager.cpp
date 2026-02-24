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
#include <utility>

namespace awviz_common
{
VisualizationManager::VisualizationManager(
  rclcpp::Node::SharedPtr node, const std::shared_ptr<rerun::RecordingStream> & stream)
: node_(std::move(node)),
  stream_(std::move(stream)),
  tf_manager_(std::make_unique<TransformationManager>(node_, stream_)),
  topic_scanner_(std::make_unique<TopicScanner>(node_)),
  display_spawner_(std::make_unique<DisplaySpawner>(node_, stream_, tf_manager_->entities()))
{
  using std::chrono_literals::operator""ms;
  stream_->log_static(TF_ROOT, rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
  parallel_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  callback_timer_ = rclcpp::create_timer(
    node_, node_->get_clock(), 100ms, [&]() { create_subscriptions(); }, parallel_callback_group_);
}

VisualizationManager::~VisualizationManager()
{
  display_registry_.clear();
}

void VisualizationManager::create_subscriptions()
{
  const auto topics = topic_scanner_->scan();
  for (const auto & [topic_name, topic_types] : topics) {
    if (display_registry_.contains(topic_name)) {
      continue;
    }

    if (topic_types.empty()) {
      display_registry_.set(topic_name, nullptr);
      continue;
    }

    const auto & topic_type = topic_types.front();
    auto display = display_spawner_->spawn(topic_name, topic_type);

    display_registry_.set(topic_name, std::move(display));
  }
}

}  // namespace awviz_common
