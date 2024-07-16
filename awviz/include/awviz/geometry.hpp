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

#ifndef AWVIZ__GEOMETRY_HPP_
#define AWVIZ__GEOMETRY_HPP_

#include <std_msgs/msg/header.hpp>

namespace awviz
{
/**
 * @brief Check whether the frame id associated with the header is "map".
 * @param header Header msg.
 */
inline bool isMapFrameId(const std_msgs::msg::Header & header)
{
  return header.frame_id == "map";
}
}  // namespace awviz
#endif  // AWVIZ__GEOMETRY_HPP_
