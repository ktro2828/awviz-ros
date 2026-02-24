// Copyright 2026 Kotaro Uetake.
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

#ifndef AWVIZ_COMMON__DISPLAY_REGISTRY_HPP_
#define AWVIZ_COMMON__DISPLAY_REGISTRY_HPP_

#include "awviz_common/display.hpp"

#include <map>
#include <memory>
#include <string>

namespace awviz_common
{
/**
 * @brief Registry to manage displays by topic name.
 *
 * This class owns display instances and provides lookup helpers.
 */
class DisplayRegistry
{
public:
  using DisplayMap = std::map<std::string, std::shared_ptr<Display>>;

  DisplayRegistry() = default;

  ~DisplayRegistry() = default;

  /**
   * @brief Check if a topic is already registered.
   * @param topic Topic name.
   * @return true if the topic exists.
   */
  bool contains(const std::string & topic) const;

  /**
   * @brief Get a display for a topic.
   * @param topic Topic name.
   * @return Shared pointer to display, or nullptr if not found.
   */
  std::shared_ptr<Display> get(const std::string & topic) const;

  /**
   * @brief Register or replace a display for a topic.
   * @param topic Topic name.
   * @param display Display instance.
   */
  void set(const std::string & topic, std::shared_ptr<Display> display);

  /**
   * @brief Return the full registry map.
   * @return Map of topic name to display.
   */
  const DisplayMap & all() const;

  /**
   * @brief Stop all displays and clear the registry.
   */
  void clear();

private:
  DisplayMap displays_;
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__DISPLAY_REGISTRY_HPP_
