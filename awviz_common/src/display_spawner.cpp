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

#include "awviz_common/display_spawner.hpp"

#include <utility>

namespace awviz_common
{
DisplaySpawner::DisplaySpawner(
  rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> stream,
  std::shared_ptr<std::unordered_map<std::string, std::string>> entity_roots)
: node_(std::move(node)),
  stream_(std::move(stream)),
  entity_roots_(std::move(entity_roots)),
  factory_()
{
}

std::shared_ptr<Display> DisplaySpawner::spawn(
  const std::string & topic, const std::string & topic_type)
{
  const auto lookup_name = resolve_lookup_name(topic_type);
  if (!lookup_name) {
    return nullptr;
  }

  auto display = factory_.create_instance(lookup_name.value());
  if (!display) {
    return nullptr;
  }

  display->initialize(node_, stream_);
  display->set_property(topic, entity_roots_);
  display->start();
  return display;
}

std::optional<std::string> DisplaySpawner::resolve_lookup_name(const std::string & topic_type) const
{
  return factory_.get_class_lookup_name(topic_type);
}
}  // namespace awviz_common
