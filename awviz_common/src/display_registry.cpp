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

#include "awviz_common/display_registry.hpp"

#include <utility>

namespace awviz_common
{
bool DisplayRegistry::contains(const std::string & topic) const
{
  return displays_.find(topic) != displays_.cend();
}

std::shared_ptr<Display> DisplayRegistry::get(const std::string & topic) const
{
  const auto itr = displays_.find(topic);
  return itr != displays_.cend() ? itr->second : nullptr;
}

void DisplayRegistry::set(const std::string & topic, std::shared_ptr<Display> display)
{
  displays_[topic] = std::move(display);
}

const DisplayRegistry::DisplayMap & DisplayRegistry::all() const
{
  return displays_;
}

void DisplayRegistry::clear()
{
  for (auto & [_, display] : displays_) {
    if (display) {
      display->end();
    }
  }
  displays_.clear();
}
}  // namespace awviz_common
