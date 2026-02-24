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

#include <chrono>
#include <utility>

namespace awviz_common
{
TransformationManager::TransformationManager(
  rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> stream)
: node_(std::move(node)),
  stream_(std::move(stream)),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
  tf_tree_(std::make_shared<TfTree>()),
  tf_tree_mtx_(std::make_shared<std::mutex>()),
  entities_mtx_(std::make_shared<std::mutex>()),
  tf_tree_updater_(std::make_unique<TfTreeUpdater>(node_, tf_buffer_, tf_tree_mtx_)),
  entity_path_resolver_(
    std::make_unique<EntityPathResolver>(tf_tree_, tf_tree_mtx_, entities_mtx_)),
  transform_logger_(std::make_unique<TransformLogger>(
    node_, stream_, tf_buffer_, entity_path_resolver_->entities(), entities_mtx_))
{
  // NOTE: Invoke callback every 100 [ms]
  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    node_, node_->get_clock(), 100ms, std::bind(&TransformationManager::timer_callback, this));
}

void TransformationManager::timer_callback()
{
  // 1. Parse all frames and update TF tree
  update_tree();

  // 2. Update entities
  update_entities();

  // 3. Log transformations in the recording stream.
  log_transforms();
}

void TransformationManager::update_tree()
{
  tf_tree_updater_->update(*tf_tree_);
}

void TransformationManager::update_entities()
{
  for (const auto & [_, frame] : tf_tree_->get_frames()) {
    entity_path_resolver_->update_entity(frame);
  }
}

void TransformationManager::log_transforms()
{
  for (const auto & [_, frame] : tf_tree_->get_frames()) {
    transform_logger_->log_transform(frame);
  }
}

}  // namespace awviz_common
