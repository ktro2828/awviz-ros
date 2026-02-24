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

#ifndef AWVIZ_COMMON__DISPLAY_SPAWNER_HPP_
#define AWVIZ_COMMON__DISPLAY_SPAWNER_HPP_

#include "awviz_common/display.hpp"
#include "awviz_common/display_factory.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace awviz_common
{
/**
 * @brief A request object to spawn a display instance.
 */
struct DisplaySpawnRequest
{
  std::string topic;
  std::string message_type;
  std::shared_ptr<std::unordered_map<std::string, std::string>> entity_roots;
};

/**
 * @brief Interface for creating display instances from topic metadata.
 */
class IDisplaySpawner
{
public:
  virtual ~IDisplaySpawner() = default;

  /**
   * @brief Spawn a display instance for the given request.
   * @param request Display spawn request.
   * @return A display instance if a plugin is available, otherwise nullptr.
   */
  virtual std::shared_ptr<Display> spawn(const DisplaySpawnRequest & request) = 0;

  /**
   * @brief Resolve a plugin lookup name for the given message type.
   * @param message_type ROS message type name.
   * @return Lookup name if a plugin is registered, otherwise std::nullopt.
   */
  virtual std::optional<std::string> resolve_lookup_name(
    const std::string & message_type) const = 0;
};

/**
 * @brief Concrete spawner that uses DisplayFactory and standard topic properties.
 */
class DisplaySpawner final : public IDisplaySpawner
{
public:
  DisplaySpawner(
    rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> stream,
    std::shared_ptr<std::unordered_map<std::string, std::string>> entity_roots);

  std::shared_ptr<Display> spawn(const DisplaySpawnRequest & request) override;

  std::optional<std::string> resolve_lookup_name(const std::string & message_type) const override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rerun::RecordingStream> stream_;
  std::shared_ptr<std::unordered_map<std::string, std::string>> entity_roots_;
  DisplayFactory factory_;
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__DISPLAY_SPAWNER_HPP_
