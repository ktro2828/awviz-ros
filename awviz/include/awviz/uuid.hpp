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

#ifndef AWVIZ__UUID_HPP_
#define AWVIZ__UUID_HPP_

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <cstdint>
#include <sstream>
#include <string>

namespace awviz
{

template <typename T>
T uuid(const unique_identifier_msgs::msg::UUID & msg);

/**
 * @brief Convert UUID msg into a single uint16_t value.
 * @param msg UUID msg.
 * @param uint16_t A single uint16_t value.
 */
template <>
uint16_t uuid<uint16_t>(const unique_identifier_msgs::msg::UUID & msg)
{
  uint16_t output = 0;
  for (size_t i = 0; i < msg.uuid.size(); ++i) {
    output |= static_cast<uint16_t>(msg.uuid[i]) << (i * 8);
  }
  return output;
}

template <>
std::string uuid<std::string>(const unique_identifier_msgs::msg::UUID & msg)
{
  std::ostringstream ss;
  for (const auto & v : msg.uuid) {
    ss << static_cast<int>(v);
  }
  return ss.str();
}
}  // namespace awviz
#endif  //  AWVIZ__UUID_HPP_
