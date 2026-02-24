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

#include "awviz_common/transformation/tf_tree_updater.hpp"

#include <yaml-cpp/yaml.h>

#include <mutex>
#include <string>
#include <utility>

namespace awviz_common
{
TfTreeUpdater::TfTreeUpdater(
  rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<std::mutex> tf_tree_mtx)
: node_(std::move(node)), tf_buffer_(std::move(tf_buffer)), tf_tree_mtx_(std::move(tf_tree_mtx))
{
}

void TfTreeUpdater::update(TfTree & tree)
{
  const auto yaml = tf_buffer_->allFramesAsYAML();
  try {
    YAML::Node frames = YAML::Load(yaml);
    for (YAML::const_iterator itr = frames.begin(); itr != frames.end(); ++itr) {
      const auto id = itr->first.as<std::string>();
      const auto parent = itr->second["parent"].as<std::string>();

      std::lock_guard<std::mutex> lock(*tf_tree_mtx_);
      tree.emplace(id, parent);
    }
  } catch (const YAML::Exception & ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to parse TF YAML: " << ex.what());
  }
}
}  // namespace awviz_common
